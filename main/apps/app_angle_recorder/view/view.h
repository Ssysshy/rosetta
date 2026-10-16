#pragma once
#include "../model/angle_recorder.h"
#include <lvgl.h>
#include <array>
#include <cstddef>
#include <cstdint>

namespace view {

class AngleRecorderView {
public:
    ~AngleRecorderView();
    void init(lv_obj_t* parent, bool sensorReady);
    uint32_t durationMs() const;
    bool takeStartRequest();
    int takePageDelta();
    void prepare(const model::angle::Stage& stage, const model::angle::Sample* samples);
    void update(const model::angle::Status& status, const model::angle::Stage& stage,
                size_t page, const char* message);

private:
    struct Point {
        uint32_t timeMs = 0;
        float value = 0.0f;
        uint32_t segment = 0;
    };
    struct Graph {
        AngleRecorderView* owner = nullptr;
        size_t axis = 0;
        lv_obj_t* object = nullptr;
        lv_obj_t* label = nullptr;
        std::array<Point, 512> points{};
        size_t count = 0;
        float minimum = -1.0f;
        float maximum = 1.0f;
        uint32_t elapsedMs = 0;
    };
    static void onDuration(lv_event_t* event);
    static void onMode(lv_event_t* event);
    static void onStart(lv_event_t* event);
    static void onGesture(lv_event_t* event);
    static void onDraw(lv_event_t* event);
    void updateDuration();
    void prepareGraph(Graph& graph, const model::angle::Stage& stage, const model::angle::Sample* samples);
    void drawGraph(Graph& graph, lv_layer_t* layer);

    lv_obj_t* _root = nullptr;
    lv_obj_t* _title = nullptr;
    lv_obj_t* _status = nullptr;
    lv_obj_t* _hint = nullptr;
    lv_obj_t* _duration_panel = nullptr;
    lv_obj_t* _duration_label = nullptr;
    lv_obj_t* _duration_hint = nullptr;
    lv_obj_t* _start_button = nullptr;
    std::array<lv_obj_t*, 2> _mode_buttons{};
    std::array<lv_obj_t*, 2> _duration_buttons{};
    std::array<Graph, 3> _graphs{};
    std::array<Graph, 3> _prepared_graphs{};
    uint32_t _duration_ms = 30000;
    bool _unlimited = false;
    bool _start_requested = false;
    int _page_delta = 0;
};

}  // namespace view
