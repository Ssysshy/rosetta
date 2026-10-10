#include "angle_recorder.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <new>
#ifdef ESP_PLATFORM
#include <esp_heap_caps.h>
#endif

namespace model::angle {
namespace {
constexpr uint64_t SamplePeriodUs = 10000;
constexpr uint64_t LostSamplingUs = 250000;
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

bool AngleRecorder::allocate()
{
    if (_samples != nullptr) {
        return true;
    }
    constexpr size_t Alignment = alignof(uint64_t);
    constexpr size_t Offset = (Capacity * sizeof(Sample) + Alignment - 1) / Alignment * Alignment;
    constexpr size_t Bytes = Offset + Capacity * sizeof(uint64_t);
#ifdef ESP_PLATFORM
    void* memory = heap_caps_malloc(Bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (memory == nullptr) {
        memory = heap_caps_malloc(Bytes, MALLOC_CAP_8BIT);
    }
#else
    void* memory = std::malloc(Bytes);
#endif
    if (memory == nullptr) {
        return false;
    }
    _samples = static_cast<Sample*>(memory);
    _timestamps = reinterpret_cast<uint64_t*>(static_cast<unsigned char*>(memory) + Offset);
    for (size_t index = 0; index < Capacity; ++index) {
        new (_samples + index) Sample{};
        new (_timestamps + index) uint64_t{0};
    }
    return true;
}

bool AngleRecorder::begin(uint32_t durationMs, uint64_t nowUs)
{
    if (_state != State::Setup) {
        return false;
    }
    if (!allocate()) {
        _notice = Notice::MemoryFull;
        return false;
    }
    _durationMs = durationMs == 0 ? 30000 : std::clamp(durationMs, uint32_t{10000}, uint32_t{300000});
    _sampleCount = 0;
    _snapshotCount = 0;
    _currentStage = {};
    _state = State::Calibrating;
    _notice = Notice::Calibrating;
    _beginUs = nowUs;
    _lastReadingUs = nowUs;
    _segmentStartUs = 0;
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
    resetCalibration();
    return true;
}

void AngleRecorder::resetCalibration()
{
    _stableCount = 0;
    _calibrationStartUs = 0;
    _accelSum = {};
    _gyroSum = {};
}

uint32_t AngleRecorder::elapsed(uint64_t nowUs) const
{
    uint64_t value = _accumulatedUs;
    if (_state == State::Recording && nowUs > _segmentStartUs) {
        value += nowUs - _segmentStartUs;
    }
    return static_cast<uint32_t>(std::min(value / 1000, uint64_t{_durationMs}));
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
    if (_sampleCount >= Capacity) {
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
    if (_state != State::Calibrating && _state != State::Recording && _state != State::Paused) {
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
    if (_hasReading && deltaUs > MaximumIntegrationUs && _state != State::Calibrating) {
        markGap(_lastReadingUs);
        stop(std::max(_lastReadingUs, _lastEventUs));
        _notice = Notice::SamplingLost;
        return;
    }
    if (_state == State::Recording &&
        reading.timestampUs > _segmentStartUs + uint64_t{_durationMs} * 1000 - _accumulatedUs) {
        stop(_segmentStartUs + uint64_t{_durationMs} * 1000 - _accumulatedUs);
        return;
    }
    if (_state == State::Calibrating) {
        const float accelNorm = magnitude(reading.accel);
        std::array<float, 3> change{};
        for (size_t axis = 0; axis < 3; ++axis) {
            change[axis] = reading.accel[axis] - _lastAccel[axis];
        }
        const bool cadence = !_hasReading || (deltaUs >= 4000 && deltaUs <= 25000);
        const bool stable = cadence && accelNorm >= 0.95F && accelNorm <= 1.05F &&
                            magnitude(reading.gyro) <= 2.0F &&
                            (_stableCount == 0 || magnitude(change) <= 0.03F);
        if (!stable) {
            resetCalibration();
            _notice = Notice::KeepStill;
        } else {
            if (_stableCount == 0) {
                _calibrationStartUs = reading.timestampUs;
            }
            ++_stableCount;
            for (size_t axis = 0; axis < 3; ++axis) {
                _accelSum[axis] += reading.accel[axis];
                _gyroSum[axis] += reading.gyro[axis];
            }
            _notice = Notice::Calibrating;
        }
        _lastAccel = reading.accel;
        _lastReadingUs = reading.timestampUs;
        _hasReading = true;
        if (_stableCount >= 100 && reading.timestampUs - _calibrationStartUs >= 990000) {
            for (size_t axis = 0; axis < 3; ++axis) {
                _accelSum[axis] /= static_cast<float>(_stableCount);
                _gyroSum[axis] /= static_cast<float>(_stableCount);
            }
            if (!_estimator.initialize(_accelSum, _gyroSum)) {
                resetCalibration();
                _notice = Notice::SensorError;
                return;
            }
            _reference = _estimator.orientation();
            _state = State::Recording;
            _notice = Notice::Ready;
            _segmentStartUs = reading.timestampUs;
            _lastEventUs = reading.timestampUs;
            _firstGapUs = UINT64_MAX;
            _breakPending = false;
            updateAngles();
            _angles = {};
            append(reading.timestampUs, 0);
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
    if (_state == State::Recording && elapsed(reading.timestampUs) >= _durationMs) {
        stop(reading.timestampUs);
    }
}

void AngleRecorder::tick(uint64_t nowUs)
{
    if (_state == State::Setup || _state == State::Finished || nowUs < _lastReadingUs) {
        return;
    }
    if ((_state == State::Calibrating && nowUs - _lastReadingUs >= LostSamplingUs) ||
        (_state != State::Calibrating && nowUs - _lastReadingUs > MaximumIntegrationUs)) {
        if (_state == State::Calibrating) {
            resetCalibration();
            _hasReading = false;
            _notice = Notice::SensorUnavailable;
        } else {
            markGap(_lastReadingUs);
            stop(std::max(_lastReadingUs, _lastEventUs));
            _notice = Notice::SamplingLost;
        }
        return;
    }
    if (_state == State::Recording && elapsed(nowUs) >= _durationMs) {
        stop(_segmentStartUs + uint64_t{_durationMs} * 1000 - _accumulatedUs);
    }
}

void AngleRecorder::samplingError(uint64_t nowUs)
{
    if (_state == State::Setup || _state == State::Finished) {
        return;
    }
    if (_state == State::Calibrating) {
        resetCalibration();
    } else {
        markGap(nowUs);
    }
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
    if (elapsed(eventUs) >= _durationMs) {
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
    if (_state == State::Recording && elapsed(eventUs) >= _durationMs) {
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
                                                                  uint64_t{_durationMs}))
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
    if ((_state != State::Calibrating && _state != State::Recording && _state != State::Paused) ||
        eventUs < _lastEventUs) {
        return;
    }
    _finalWasRecording = _state == State::Recording;
    _finalSegmentAccumulatedUs = _accumulatedUs;
    if (_finalWasRecording) {
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
