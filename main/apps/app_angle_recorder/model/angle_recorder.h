#pragma once
#include "attitude_estimator.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace model::angle {

enum class State { Setup, Recording, Paused, Finished };
enum class Notice {
    None, SensorUnavailable, MemoryFull, SnapshotFull, SnapshotSaved,
    SamplingLost, SensorError, Singular, QueueFull
};

struct ImuReading {
    std::array<float, 3> accel;
    std::array<float, 3> gyro;
    uint64_t timestampUs;
};

struct Range {
    float minimum = 0.0F;
    float maximum = 0.0F;
    bool valid = false;
};

struct Sample {
    uint32_t timeMs;
    std::array<float, 3> angles;
    uint8_t validMask;
    bool breakBefore;
};

struct Stage {
    uint32_t elapsedMs;
    size_t sampleCount;
    std::array<Range, 3> ranges;
    bool degraded;
};

struct Status {
    State state;
    Notice notice;
    uint32_t durationMs;
    uint32_t elapsedMs;
    size_t snapshotCount;
    std::array<float, 3> angles;
    uint8_t validMask;
};

class AngleRecorder {
public:
    AngleRecorder() = default;
    ~AngleRecorder();
    AngleRecorder(const AngleRecorder&) = delete;
    AngleRecorder& operator=(const AngleRecorder&) = delete;
    bool begin(uint32_t durationMs, uint64_t nowUs);
    void reset();
    void ingest(const ImuReading& reading);
    void tick(uint64_t nowUs);
    void samplingError(uint64_t nowUs);
    void pause(uint64_t eventUs);
    void resume(uint64_t eventUs);
    bool snapshot(uint64_t eventUs);
    void stop(uint64_t eventUs);
    Status status(uint64_t nowUs) const;
    Stage stage(size_t index, uint64_t nowUs) const;
    const Sample* samples() const;
    size_t sampleCount() const;

private:
    static constexpr size_t Capacity = 33002;
    uint32_t elapsed(uint64_t nowUs) const;
    size_t prefix(uint64_t eventUs) const;
    Stage summarize(size_t count, uint32_t elapsedMs, uint64_t eventUs) const;
    void truncate(uint64_t eventUs);
    void append(uint64_t timestampUs, uint32_t timeMs);
    void markGap(uint64_t timestampUs);
    void updateAngles();
    bool allocate(size_t capacity);

    Sample* _samples = nullptr;
    uint64_t* _timestamps = nullptr;
    size_t _capacity = 0;
    size_t _sampleCount = 0;
    std::array<Stage, 32> _snapshots{};
    Stage _currentStage{};
    size_t _snapshotCount = 0;
    State _state = State::Setup;
    Notice _notice = Notice::None;
    uint32_t _durationMs = 30000;
    uint64_t _beginUs = 0;
    uint64_t _lastReadingUs = 0;
    uint64_t _segmentStartUs = 0;
    uint64_t _accumulatedUs = 0;
    uint64_t _firstGapUs = UINT64_MAX;
    uint64_t _lastStoredUs = 0;
    uint64_t _lastEventUs = 0;
    uint64_t _finalEndUs = 0;
    uint64_t _finalSegmentAccumulatedUs = 0;
    uint64_t _lastSnapshotUs = 0;
    bool _finalWasRecording = false;
    bool _hasReading = false;
    bool _breakPending = false;
    bool _unwrapValid = false;
    bool _singular = false;
    std::array<float, 3> _angles{};
    std::array<float, 3> _rawPrevious{};
    uint8_t _validMask = 0;
    AttitudeEstimator _estimator;
    Quaternion _reference{1.0F, 0.0F, 0.0F, 0.0F};
};

}  // namespace model::angle
