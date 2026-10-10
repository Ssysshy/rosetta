#include "app_angle_recorder.h"
#include <assets/assets.h>
#include <hal/hal.h>
#include <esp_timer.h>
#include <algorithm>
#include <new>

using model::angle::State;
using model::angle::Notice;

namespace {
uint64_t currentUs()
{
    return static_cast<uint64_t>(esp_timer_get_time());
}

const char* noticeText(Notice notice)
{
    switch (notice) {
        case Notice::SensorUnavailable: return "IMU unavailable";
        case Notice::KeepStill: return "Keep device still to calibrate";
        case Notice::MemoryFull: return "Insufficient recording memory";
        case Notice::SnapshotFull: return "Snapshot limit reached (32)";
        case Notice::SamplingLost: return "Sampling interrupted - record ended";
        case Notice::SensorError: return "Sensor error - data has gaps";
        case Notice::Singular: return "X/Z unavailable near pitch 90 deg";
        default: return nullptr;
    }
}
}  // namespace

AppAngleRecorder::AppAngleRecorder()
{
    setAppInfo().name = "AngleRecorder";
    setAppInfo().icon = const_cast<lv_image_dsc_t*>(&icon_angle_recorder);
}

void AppAngleRecorder::onOpen()
{
    _shutdown.store(false);
    _last_state = State::Setup;
    _last_notice = Notice::None;
    _last_snapshot_count = 0;
    _page = 0;
    _b_active = false;
    _b_long_handled = false;
    _chord_active = false;
    _message = nullptr;
    _last_refresh = 0;
    _keys.reset(new (std::nothrow) input::KeyManager());
    _recorder.reset(new (std::nothrow) model::angle::AngleRecorder());
    _mutex = xSemaphoreCreateMutex();
    _stopped = xSemaphoreCreateBinary();
    _commands = xQueueCreate(8, sizeof(Command));
    if (_keys && _recorder && _mutex && _stopped && _commands) {
        if (xTaskCreate(runWorker, "angle_sampler", 6144, this, 2, &_task) != pdPASS) {
            _task = nullptr;
        }
    }
    {
        LvglLockGuard lock;
        _view.reset(new (std::nothrow) view::AngleRecorderView());
        if (_view) {
            _view->init(lv_screen_active(), GetHAL().isImuReady());
        }
    }
    if (_task == nullptr) {
        showMessage("Recorder initialization failed", GetHAL().millis());
    }
}

void AppAngleRecorder::showMessage(const char* message, uint32_t now)
{
    _message = message;
    _message_tick = now;
}

bool AppAngleRecorder::sendCommand(CommandType type, uint64_t timestampUs, uint32_t durationMs)
{
    if (_task == nullptr || _commands == nullptr) {
        showMessage("Recorder initialization failed", GetHAL().millis());
        return false;
    }
    const Command command{type, timestampUs, durationMs};
    if (xQueueSend(_commands, &command, 0) != pdTRUE) {
        showMessage("Operation failed - please retry", GetHAL().millis());
        return false;
    }
    return true;
}

void AppAngleRecorder::onRunning()
{
    GetHAL().updateButtonStates();
    if ((_keys && _keys->update(false) == input::KeyEvent::GoHome) ||
        (!_keys && GetHAL().btnA.isHolding() && GetHAL().btnB.isHolding())) {
        close();
        return;
    }
    if (!_view) {
        close();
        return;
    }
    const uint32_t now = GetHAL().millis();
    const uint64_t now_us = currentUs();
    model::angle::Status status{State::Setup, Notice::None, 30000, 0, 0, {}, 0};
    if (_mutex && _recorder) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        status = _recorder->status(now_us);
        xSemaphoreGive(_mutex);
    }
    if (GetHAL().btnA.isPressed() && GetHAL().btnB.isPressed()) {
        _chord_active = true;
    }
    const bool suppress_keys = _chord_active;
    if (GetHAL().btnA.isReleased() && GetHAL().btnB.isReleased()) {
        _chord_active = false;
    }
    if (!suppress_keys && GetHAL().btnA.wasClicked()) {
        if (status.state == State::Setup) {
            uint32_t duration = 30000;
            {
                LvglLockGuard lock;
                duration = _view->durationMs();
            }
            if (!GetHAL().isImuReady()) {
                showMessage("IMU unavailable", now);
            } else {
                sendCommand(CommandType::Start, now_us, duration);
            }
        } else if (status.state == State::Recording) {
            sendCommand(CommandType::Pause, now_us);
        } else if (status.state == State::Paused) {
            sendCommand(CommandType::Resume, now_us);
        }
    }
    if (GetHAL().btnB.wasPressed()) {
        _b_active = true;
        _b_long_handled = false;
        _b_pressed_us = now_us;
        _b_press_state = status.state;
    }
    if (_b_active && !_b_long_handled && !suppress_keys && GetHAL().btnA.isReleased() &&
        GetHAL().btnB.pressedFor(800)) {
        _b_long_handled = true;
        if (status.state == State::Paused && _b_press_state == State::Paused) {
            _b_long_handled = sendCommand(CommandType::Stop, now_us);
        }
    }
    if (_b_active && GetHAL().btnB.wasReleased()) {
        if (!suppress_keys && !_b_long_handled && now_us - _b_pressed_us < 800000 &&
            (status.state == State::Recording || status.state == State::Finished) && _b_press_state == State::Recording) {
            sendCommand(CommandType::Snapshot, _b_pressed_us);
        }
        _b_active = false;
    }
    if (status.state == State::Finished && _last_state != State::Finished) {
        _page = status.snapshotCount;
    }
    if (status.snapshotCount > _last_snapshot_count) {
        if (status.state == State::Finished && _page == _last_snapshot_count) {
            _page = status.snapshotCount;
        }
        showMessage("Snapshot saved", now);
    }
    if (status.notice != _last_notice && noticeText(status.notice) != nullptr) {
        showMessage(noticeText(status.notice), now);
    }
    _last_notice = status.notice;
    const bool state_changed = status.state != _last_state;
    _last_state = status.state;
    _last_snapshot_count = status.snapshotCount;
    int page_delta = 0;
    {
        LvglLockGuard lock;
        page_delta = _view->takePageDelta();
    }
    if (status.state == State::Finished && page_delta != 0) {
        const int next_page = static_cast<int>(_page) + page_delta;
        _page = static_cast<size_t>(std::clamp(next_page, 0, static_cast<int>(status.snapshotCount)));
    }
    if (now - _last_refresh < 100 && !state_changed && page_delta == 0) {
        GetHAL().delay(2);
        return;
    }
    _last_refresh = now;
    model::angle::Stage stage{};
    const model::angle::Sample* samples = nullptr;
    if (_mutex && _recorder) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        status = _recorder->status(currentUs());
        if (status.state != State::Finished) {
            _page = status.snapshotCount;
        } else if (_last_state != State::Finished) {
            _page = status.snapshotCount;
        }
        stage = _recorder->stage(_page, currentUs());
        samples = _recorder->samples();
        _view->prepare(stage, samples);
        xSemaphoreGive(_mutex);
    }
    const char* message = status.state == State::Calibrating || status.state == State::Setup
                              ? noticeText(status.notice) : nullptr;
    if (_message != nullptr && (now - _message_tick < 1800 || _task == nullptr)) {
        message = _message;
    } else if (!GetHAL().isImuReady()) {
        message = "IMU unavailable";
    }
    {
        LvglLockGuard lock;
        _view->update(status, stage, _page, message);
    }
    GetHAL().delay(2);
}

void AppAngleRecorder::runWorker(void* argument)
{
    static_cast<AppAngleRecorder*>(argument)->worker();
}

void AppAngleRecorder::worker()
{
    const TickType_t period = std::max<TickType_t>(1, pdMS_TO_TICKS(5));
    TickType_t wake_tick = xTaskGetTickCount();
    while (!_shutdown.load()) {
        Command command;
        while (xQueueReceive(_commands, &command, 0) == pdTRUE) {
            xSemaphoreTake(_mutex, portMAX_DELAY);
            switch (command.type) {
                case CommandType::Start:
                    _recorder->begin(command.durationMs, command.timestampUs);
                    break;
                case CommandType::Pause: _recorder->pause(command.timestampUs); break;
                case CommandType::Resume: _recorder->resume(command.timestampUs); break;
                case CommandType::Snapshot: _recorder->snapshot(command.timestampUs); break;
                case CommandType::Stop: _recorder->stop(command.timestampUs); break;
            }
            xSemaphoreGive(_mutex);
        }
        xSemaphoreTake(_mutex, portMAX_DELAY);
        const State state = _recorder->status(currentUs()).state;
        xSemaphoreGive(_mutex);
        if (state == State::Calibrating || state == State::Recording || state == State::Paused) {
            Hal::ImuSample sample;
            const Hal::ImuSampleResult result = GetHAL().readImuSample(sample);
            xSemaphoreTake(_mutex, portMAX_DELAY);
            if (result == Hal::ImuSampleResult::Ready) {
                const auto& data = sample.data;
                _recorder->ingest({{data.accelX, data.accelY, data.accelZ},
                                    {data.gyroX, data.gyroY, data.gyroZ}, sample.timestampUs});
            } else if (result == Hal::ImuSampleResult::Error) {
                _recorder->samplingError(currentUs());
            }
            _recorder->tick(currentUs());
            xSemaphoreGive(_mutex);
        }
        const TickType_t now_tick = xTaskGetTickCount();
        if (now_tick - wake_tick >= period) {
            wake_tick = now_tick;
        }
        vTaskDelayUntil(&wake_tick, period);
    }
    xSemaphoreGive(_stopped);
    vTaskDelete(nullptr);
}

void AppAngleRecorder::releaseWorker()
{
    if (_task != nullptr) {
        _shutdown.store(true);
        xSemaphoreTake(_stopped, portMAX_DELAY);
        _task = nullptr;
    }
    if (_commands != nullptr) {
        vQueueDelete(_commands);
        _commands = nullptr;
    }
    if (_mutex != nullptr) {
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }
    if (_stopped != nullptr) {
        vSemaphoreDelete(_stopped);
        _stopped = nullptr;
    }
}

void AppAngleRecorder::onClose()
{
    releaseWorker();
    {
        LvglLockGuard lock;
        _view.reset();
    }
    _keys.reset();
    _recorder.reset();
}
