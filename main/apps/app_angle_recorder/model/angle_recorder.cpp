#include "angle_recorder.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>
#ifdef ESP_PLATFORM
#include <esp_heap_caps.h>
#endif

namespace model::angle {
namespace {
constexpr uint64_t SamplePeriodUs = 10000;
constexpr uint64_t MaximumIntegrationUs = 100000;

float magnitude(const std::array<float, 3>& values)
{
    return std::sqrt(values[0] * values[0] + values[1] * values[1] + values[2] * values[2]);
}

bool finite(const ImuReading& reading)
{
    for (size_t axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(reading.accel[axis]) || !std::isfinite(reading.gyro[axis]) ||
            std::abs(reading.gyro[axis]) >= 1990.0F) {
            return false;
        }
    }
    return true;
}

void include(Stage& stage, const Sample& sample)
{
    stage.degraded = stage.degraded || sample.validMask != 0x07;
    for (size_t axis = 0; axis < 3; ++axis) {
        if ((sample.validMask & (1U << axis)) == 0) {
            continue;
        }
        Range& range = stage.ranges[axis];
        if (!range.valid) {
            range = {sample.angles[axis], sample.angles[axis], true};
        } else {
            range.minimum = std::min(range.minimum, sample.angles[axis]);
            range.maximum = std::max(range.maximum, sample.angles[axis]);
        }
    }
}
}  // namespace

AngleRecorder::~AngleRecorder()
{
    std::free(_samples);
}

bool AngleRecorder::allocate(size_t capacity)
{
    if (capacity <= _capacity) {
        return true;
    }
    constexpr size_t Alignment = alignof(uint64_t);
    if (capacity > (std::numeric_limits<size_t>::max() - Alignment + 1) / (sizeof(Sample) + sizeof(uint64_t))) {
        return false;
    }
    const size_t offset = (capacity * sizeof(Sample) + Alignment - 1) / Alignment * Alignment;
    const size_t bytes = offset + capacity * sizeof(uint64_t);
    const size_t old_offset = (_capacity * sizeof(Sample) + Alignment - 1) / Alignment * Alignment;
#ifdef ESP_PLATFORM
    void* memory = heap_caps_realloc(_samples, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (memory == nullptr) {
        memory = heap_caps_realloc(_samples, bytes, MALLOC_CAP_8BIT);
    }
#else
    void* memory = std::realloc(_samples, bytes);
#endif
    if (memory == nullptr) {
        return false;
    }
    _samples = static_cast<Sample*>(memory);
    _timestamps = reinterpret_cast<uint64_t*>(static_cast<unsigned char*>(memory) + offset);
    if (_sampleCount > 0) {
        std::memmove(_timestamps, static_cast<unsigned char*>(memory) + old_offset,
                     _sampleCount * sizeof(uint64_t));
    }
    for (size_t index = _capacity; index < capacity; ++index) {
        new (_samples + index) Sample{};
        new (_timestamps + index) uint64_t{0};
    }
    _capacity = capacity;
    return true;
}

bool AngleRecorder::begin(uint32_t durationMs, uint64_t nowUs)
{
    if (_state != State::Setup) {
        return false;
    }
    if (!allocate(Capacity)) {
        _notice = Notice::MemoryFull;
        return false;
    }
    _durationMs = durationMs == 0 ? 0 : std::clamp(durationMs, uint32_t{10000}, uint32_t{300000});
    _sampleCount = 0;
    _snapshotCount = 0;
    _currentStage = {};
    _state = State::Recording;
    _notice = Notice::None;
    _beginUs = nowUs;
    _lastReadingUs = nowUs;
    _segmentStartUs = nowUs;
    _accumulatedUs = 0;
    _firstGapUs = UINT64_MAX;
    _lastStoredUs = 0;
    _lastEventUs = nowUs;
    _lastSnapshotUs = 0;
    _finalEndUs = 0;
    _finalSegmentAccumulatedUs = 0;
    _finalWasRecording = false;
    _hasReading = false;
    _breakPending = false;
    _unwrapValid = false;
    _singular = false;
    _angles = {};
    _validMask = 0;
    return true;
}

void AngleRecorder::reset()
{
    if (_state != State::Finished) {
        return;
    }
    std::free(_samples);
    _samples = nullptr;
    _timestamps = nullptr;
    _capacity = 0;
    _sampleCount = 0;
    _snapshotCount = 0;
    _currentStage = {};
    _accumulatedUs = 0;
    _firstGapUs = UINT64_MAX;
    _angles = {};
    _validMask = 0;
    _notice = Notice::None;
    _state = State::Setup;
}

uint32_t AngleRecorder::elapsed(uint64_t nowUs) const
{
    uint64_t value = _accumulatedUs;
    if (_state == State::Recording && nowUs > _segmentStartUs) {
        value += nowUs - _segmentStartUs;
    }
    const uint32_t limit = _durationMs == 0 ? UINT32_MAX : _durationMs;
    return static_cast<uint32_t>(std::min(value / 1000, uint64_t{limit}));
}

void AngleRecorder::markGap(uint64_t timestampUs)
{
    _firstGapUs = std::min(_firstGapUs, timestampUs);
    _breakPending = true;
}

void AngleRecorder::updateAngles()
{
    const std::array<float, 3> raw = AttitudeEstimator::relativeAngles(_reference, _estimator.orientation());
    const bool singular = std::abs(raw[1]) >= (_singular ? 85.0F : 87.0F);
    if (singular != _singular) {
        _unwrapValid = false;
    }
    _singular = singular;
    _validMask = singular ? 0x02 : 0x07;
    if (singular) {
        _angles = raw;
        _unwrapValid = false;
        _notice = Notice::Singular;
    } else {
        for (size_t axis = 0; axis < 3; ++axis) {
            if (axis == 1 || !_unwrapValid) {
                _angles[axis] = raw[axis];
            } else {
                _angles[axis] += std::remainder(raw[axis] - _rawPrevious[axis], 360.0F);
            }
        }
        _unwrapValid = true;
        if (_notice == Notice::Singular) {
            _notice = Notice::None;
        }
    }
    _rawPrevious = raw;
}

void AngleRecorder::append(uint64_t timestampUs, uint32_t timeMs)
{
    if (_sampleCount >= _capacity &&
        (_durationMs != 0 || _capacity > std::numeric_limits<size_t>::max() - Capacity ||
         !allocate(_capacity + Capacity))) {
        stop(timestampUs);
        _notice = Notice::MemoryFull;
        return;
    }
    _samples[_sampleCount] = Sample{timeMs, _angles, _validMask, _breakPending};
    include(_currentStage, _samples[_sampleCount]);
    _timestamps[_sampleCount] = timestampUs;
    ++_sampleCount;
    _lastStoredUs = timestampUs;
    _breakPending = false;
}

void AngleRecorder::ingest(const ImuReading& reading)
{
    if (_state != State::Recording && _state != State::Paused) {
        return;
    }
    if (reading.timestampUs < _beginUs || (_hasReading && reading.timestampUs <= _lastReadingUs)) {
        return;
    }
    if (!finite(reading) || magnitude(reading.accel) < 0.01F || magnitude(reading.accel) > 32.0F) {
        samplingError(reading.timestampUs);
        return;
    }
    const uint64_t deltaUs = reading.timestampUs - _lastReadingUs;
    if (_hasReading && deltaUs > MaximumIntegrationUs) {
        markGap(_lastReadingUs);
        stop(std::max(_lastReadingUs, _lastEventUs));
        _notice = Notice::SamplingLost;
        return;
    }
    if (_state == State::Recording && _durationMs != 0 &&
        reading.timestampUs > _segmentStartUs + uint64_t{_durationMs} * 1000 - _accumulatedUs) {
        stop(_segmentStartUs + uint64_t{_durationMs} * 1000 - _accumulatedUs);
        return;
    }
    if (!_hasReading) {
        if (!_estimator.initialize(reading.accel)) {
            samplingError(reading.timestampUs);
            return;
        }
        _reference = _estimator.orientation();
        _lastReadingUs = reading.timestampUs;
        _hasReading = true;
        _notice = Notice::None;
        updateAngles();
        _angles = {};
        if (_state == State::Recording) {
            append(reading.timestampUs, elapsed(reading.timestampUs));
        }
        return;
    }
    if (deltaUs > 15000) {
        markGap(_lastReadingUs + SamplePeriodUs);
    }
    if (!_estimator.update(reading.accel, reading.gyro, static_cast<float>(deltaUs) / 1000000.0F)) {
        samplingError(reading.timestampUs);
        return;
    }
    _lastReadingUs = reading.timestampUs;
    _hasReading = true;
    if (_notice == Notice::SensorError || _notice == Notice::SensorUnavailable) {
        _notice = Notice::None;
    }
    updateAngles();
    if (_state == State::Recording && reading.timestampUs >= _segmentStartUs) {
        if (_singular) {
            _firstGapUs = std::min(_firstGapUs, reading.timestampUs);
        }
        append(reading.timestampUs, elapsed(reading.timestampUs));
    }
    if (_state == State::Recording && _durationMs != 0 && elapsed(reading.timestampUs) >= _durationMs) {
        stop(reading.timestampUs);
    }
}

void AngleRecorder::tick(uint64_t nowUs)
{
    if (_state == State::Setup || _state == State::Finished || nowUs < _lastReadingUs) {
        return;
    }
    if (nowUs - _lastReadingUs > MaximumIntegrationUs) {
        markGap(_lastReadingUs);
        stop(std::max(_lastReadingUs, _lastEventUs));
        _notice = Notice::SamplingLost;
        return;
    }
    if (_state == State::Recording && _durationMs != 0 && elapsed(nowUs) >= _durationMs) {
        stop(_segmentStartUs + uint64_t{_durationMs} * 1000 - _accumulatedUs);
    }
}

void AngleRecorder::samplingError(uint64_t nowUs)
{
    if (_state == State::Setup || _state == State::Finished) {
        return;
    }
    markGap(nowUs);
    _notice = Notice::SensorError;
    tick(nowUs);
}

size_t AngleRecorder::prefix(uint64_t eventUs) const
{
    if (_sampleCount == 0) {
        return 0;
    }
    return static_cast<size_t>(std::upper_bound(_timestamps, _timestamps + _sampleCount, eventUs) - _timestamps);
}

Stage AngleRecorder::summarize(size_t count, uint32_t elapsedMs, uint64_t eventUs) const
{
    Stage result{elapsedMs, count, {}, _firstGapUs <= eventUs};
    for (size_t index = 0; index < count; ++index) {
        include(result, _samples[index]);
    }
    return result;
}

void AngleRecorder::truncate(uint64_t eventUs)
{
    const size_t count = prefix(eventUs);
    if (count != _sampleCount) {
        _sampleCount = count;
        _currentStage = summarize(count, elapsed(eventUs), eventUs);
    }
    _lastStoredUs = count == 0 ? 0 : _timestamps[count - 1];
}

void AngleRecorder::pause(uint64_t eventUs)
{
    if (_state != State::Recording || eventUs < _lastEventUs) {
        return;
    }
    if (_durationMs != 0 && elapsed(eventUs) >= _durationMs) {
        stop(eventUs);
        return;
    }
    _accumulatedUs += eventUs - _segmentStartUs;
    _lastEventUs = eventUs;
    truncate(eventUs);
    _state = State::Paused;
    _breakPending = true;
}

void AngleRecorder::resume(uint64_t eventUs)
{
    if (_state != State::Paused || eventUs < _lastEventUs) {
        return;
    }
    _segmentStartUs = eventUs;
    _lastEventUs = eventUs;
    _state = State::Recording;
    _breakPending = true;
}

bool AngleRecorder::snapshot(uint64_t eventUs)
{
    const bool finishedSnapshot = _state == State::Finished && _finalWasRecording &&
                                  eventUs >= _segmentStartUs && eventUs <= _finalEndUs &&
                                  eventUs >= _lastSnapshotUs;
    if (!finishedSnapshot && (_state != State::Recording || eventUs < _lastEventUs)) {
        return false;
    }
    if (_state == State::Recording && _durationMs != 0 && elapsed(eventUs) >= _durationMs) {
        stop(_segmentStartUs + uint64_t{_durationMs} * 1000 - _accumulatedUs);
        if (eventUs > _finalEndUs) {
            return false;
        }
    }
    if (_snapshotCount >= _snapshots.size()) {
        _notice = Notice::SnapshotFull;
        return false;
    }
    const uint32_t elapsedMs = _state == State::Finished
                                   ? static_cast<uint32_t>(std::min((_finalSegmentAccumulatedUs + eventUs -
                                                                   _segmentStartUs) / 1000,
                                                                  uint64_t{_durationMs == 0 ? UINT32_MAX : _durationMs}))
                                   : elapsed(eventUs);
    _snapshots[_snapshotCount] = summarize(prefix(eventUs), elapsedMs, eventUs);
    _lastSnapshotUs = eventUs;
    if (_state != State::Finished) {
        _lastEventUs = eventUs;
    }
    ++_snapshotCount;
    _notice = Notice::SnapshotSaved;
    return true;
}

void AngleRecorder::stop(uint64_t eventUs)
{
    if ((_state != State::Recording && _state != State::Paused) ||
        eventUs < _lastEventUs) {
        return;
    }
    _finalWasRecording = _state == State::Recording;
    _finalSegmentAccumulatedUs = _accumulatedUs;
    if (_finalWasRecording && _durationMs != 0) {
        eventUs = std::min(eventUs, _segmentStartUs + uint64_t{_durationMs} * 1000 - _accumulatedUs);
    }
    _finalEndUs = eventUs;
    truncate(eventUs);
    _accumulatedUs = uint64_t{elapsed(eventUs)} * 1000;
    _lastEventUs = eventUs;
    _state = State::Finished;
}

Status AngleRecorder::status(uint64_t nowUs) const
{
    return {_state, _notice, _durationMs, elapsed(nowUs), _snapshotCount, _angles, _validMask};
}

Stage AngleRecorder::stage(size_t index, uint64_t nowUs) const
{
    if (index < _snapshotCount) {
        return _snapshots[index];
    }
    if (index == _snapshotCount) {
        Stage result = _currentStage;
        result.elapsedMs = elapsed(nowUs);
        result.sampleCount = _sampleCount;
        result.degraded = result.degraded || _firstGapUs <= nowUs;
        return result;
    }
    return {};
}

const Sample* AngleRecorder::samples() const
{
    return _samples;
}

size_t AngleRecorder::sampleCount() const
{
    return _sampleCount;
}

}  // namespace model::angle
