#pragma once
#include "model/angle_recorder.h"
#include "view/view.h"
#include <apps/common/key_manager/key_manager.h>
#include <mooncake.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <atomic>
#include <memory>

class AppAngleRecorder : public mooncake::AppAbility {
public:
    AppAngleRecorder();
    void onOpen() override;
    void onRunning() override;
    void onClose() override;

private:
    enum class CommandType { Start, Pause, Resume, Snapshot, Stop, Reset };
    struct Command {
        CommandType type;
        uint64_t timestampUs;
        uint32_t durationMs = 0;
    };
    static void runWorker(void* argument);
    void worker();
    bool sendCommand(CommandType type, uint64_t timestampUs, uint32_t durationMs = 0);
    void showMessage(const char* message, uint32_t now);
    void releaseWorker();

    std::unique_ptr<view::AngleRecorderView> _view;
    std::unique_ptr<input::KeyManager> _keys;
    std::unique_ptr<model::angle::AngleRecorder> _recorder;
    SemaphoreHandle_t _mutex = nullptr;
    SemaphoreHandle_t _stopped = nullptr;
    QueueHandle_t _commands = nullptr;
    TaskHandle_t _task = nullptr;
    std::atomic<bool> _shutdown{false};
    model::angle::State _last_state = model::angle::State::Setup;
    model::angle::Notice _last_notice = model::angle::Notice::None;
    model::angle::State _b_press_state = model::angle::State::Setup;
    size_t _last_snapshot_count = 0;
    size_t _page = 0;
    uint32_t _last_refresh = 0;
    uint64_t _b_pressed_us = 0;
    bool _b_active = false;
    bool _b_long_handled = false;
    bool _chord_active = false;
    const char* _message = nullptr;
    uint32_t _message_tick = 0;
};
