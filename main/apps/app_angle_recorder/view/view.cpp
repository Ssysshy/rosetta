#include "view.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace view {
namespace {
constexpr uint32_t background = 0x101923;
constexpr uint32_t foreground = 0xE5EEF9;
constexpr std::array<uint32_t, 3> colors = {0xFFB55C, 0x6DDFB1, 0x7EBCFF};
constexpr std::array<size_t, 3> display_order = {2, 0, 1};
constexpr std::array<const char*, 3> axis_names = {"X", "Y", "Z"};

lv_obj_t* makeLabel(lv_obj_t* parent, const char* text, int32_t width, int32_t y)
{
    lv_obj_t* label = lv_label_create(parent);
    lv_obj_set_width(label, width);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(foreground), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_label_set_text(label, text);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, y);
    lv_obj_add_flag(label, LV_OBJ_FLAG_GESTURE_BUBBLE);
    return label;
}
}  // namespace

AngleRecorderView::~AngleRecorderView()
{
    if (_root != nullptr) {
        lv_obj_delete(_root);
    }
}

void AngleRecorderView::init(lv_obj_t* parent, bool sensorReady)
{
    _root = lv_obj_create(parent);
    lv_obj_remove_style_all(_root);
    lv_obj_set_size(_root, 466, 466);
    lv_obj_center(_root);
    lv_obj_set_style_bg_color(_root, lv_color_hex(background), 0);
    lv_obj_set_style_bg_opa(_root, LV_OPA_COVER, 0);
    lv_obj_remove_flag(_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(_root, onGesture, LV_EVENT_GESTURE, this);
    _title = makeLabel(_root, "Angle Recorder", 240, 38);
    lv_obj_set_style_text_font(_title, &lv_font_montserrat_22, 0);
    _status = makeLabel(_root, sensorReady ? "Set recording time" : "IMU unavailable", 300, 70);
    _hint = makeLabel(_root, "A: start   Hold A+B: home", 330, 375);
    _duration_panel = lv_obj_create(_root);
    lv_obj_remove_style_all(_duration_panel);
    lv_obj_set_size(_duration_panel, 360, 230);
    lv_obj_align(_duration_panel, LV_ALIGN_CENTER, 0, 5);
    lv_obj_remove_flag(_duration_panel, LV_OBJ_FLAG_SCROLLABLE);
    _duration_label = makeLabel(_duration_panel, "30 s", 220, 20);
    lv_obj_set_style_text_font(_duration_label, &lv_font_montserrat_36, 0);
    for (int direction : {-1, 1}) {
        lv_obj_t* button = lv_button_create(_duration_panel);
        lv_obj_set_size(button, 100, 75);
        lv_obj_align(button, LV_ALIGN_CENTER, direction * 85, 40);
        lv_obj_set_style_bg_color(button, lv_color_hex(0x29435F), 0);
        lv_obj_set_style_radius(button, 24, 0);
        lv_obj_set_user_data(button, this);
        lv_obj_add_event_cb(button, onDuration, LV_EVENT_CLICKED,
                            reinterpret_cast<void*>(static_cast<intptr_t>(direction)));
        lv_obj_t* label = lv_label_create(button);
        lv_label_set_text(label, direction < 0 ? "- 10" : "+ 10");
        lv_obj_center(label);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_22, 0);
    }
    makeLabel(_duration_panel, "10 - 300 seconds", 260, 195);
    for (size_t row = 0; row < _graphs.size(); ++row) {
        Graph& graph = _graphs[row];
        graph.owner = this;
        graph.axis = display_order[row];
        graph.object = lv_obj_create(_root);
        lv_obj_remove_style_all(graph.object);
        lv_obj_set_size(graph.object, 326, 88);
        lv_obj_align(graph.object, LV_ALIGN_TOP_MID, 0, 101 + static_cast<int32_t>(row) * 90);
        lv_obj_remove_flag(graph.object, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(graph.object, LV_OBJ_FLAG_GESTURE_BUBBLE);
        lv_obj_add_flag(graph.object, LV_OBJ_FLAG_HIDDEN);
        graph.label = makeLabel(graph.object, axis_names[graph.axis], 320, 0);
        lv_obj_add_event_cb(graph.object, onDraw, LV_EVENT_DRAW_MAIN, &graph);
    }
}

uint32_t AngleRecorderView::durationMs() const
{
    return _duration_ms;
}

int AngleRecorderView::takePageDelta()
{
    const int delta = _page_delta;
    _page_delta = 0;
    return delta;
}

void AngleRecorderView::onDuration(lv_event_t* event)
{
    lv_obj_t* button = lv_event_get_target_obj(event);
    auto* self = static_cast<AngleRecorderView*>(lv_obj_get_user_data(button));
    const auto direction = static_cast<int32_t>(reinterpret_cast<intptr_t>(lv_event_get_user_data(event)));
    const int32_t duration = static_cast<int32_t>(self->_duration_ms) + direction * 10000;
    self->_duration_ms = static_cast<uint32_t>(std::clamp<int32_t>(duration, 10000, 300000));
    self->updateDuration();
}

void AngleRecorderView::updateDuration()
{
    lv_label_set_text_fmt(_duration_label, "%lu s", static_cast<unsigned long>(_duration_ms / 1000));
}

void AngleRecorderView::onGesture(lv_event_t* event)
{
    auto* self = static_cast<AngleRecorderView*>(lv_event_get_user_data(event));
    lv_indev_t* input = lv_indev_active();
    if (input == nullptr) {
        return;
    }
    const lv_dir_t direction = lv_indev_get_gesture_dir(input);
    if (direction == LV_DIR_LEFT) {
        self->_page_delta = 1;
    } else if (direction == LV_DIR_RIGHT) {
        self->_page_delta = -1;
    }
}

void AngleRecorderView::prepare(const model::angle::Stage& stage, const model::angle::Sample* samples)
{
    for (size_t row = 0; row < _prepared_graphs.size(); ++row) {
        _prepared_graphs[row].axis = display_order[row];
        prepareGraph(_prepared_graphs[row], stage, samples);
    }
}

void AngleRecorderView::update(const model::angle::Status& status, const model::angle::Stage& stage,
                               size_t page, const char* message)
{
    using model::angle::State;
    const bool setup = status.state == State::Setup;
    const bool finished = status.state == State::Finished;
    if (setup) {
        lv_obj_remove_flag(_duration_panel, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(_duration_panel, LV_OBJ_FLAG_HIDDEN);
    }
    char text[128];
    const char* state_name = "Set recording time";
    if (status.state == State::Calibrating) {
        state_name = "Calibrating: keep still";
    } else if (status.state == State::Recording) {
        state_name = "Recording";
    } else if (status.state == State::Paused) {
        state_name = "Paused";
    } else if (finished) {
        state_name = page == status.snapshotCount ? "Final result" : "Snapshot";
    }
    if (setup || status.state == State::Calibrating) {
        lv_label_set_text(_status, state_name);
    } else {
        std::snprintf(text, sizeof(text), "%s  %.1f / %.0f s", state_name,
                      static_cast<double>(stage.elapsedMs) / 1000.0,
                      static_cast<double>(status.durationMs) / 1000.0);
        lv_label_set_text(_status, text);
    }
    if (message != nullptr && message[0] != '\0') {
        lv_label_set_text(_hint, message);
    } else if (finished) {
        std::snprintf(text, sizeof(text), "%u / %u  Swipe to browse%s\nHold A+B: home",
                      static_cast<unsigned>(page + 1), static_cast<unsigned>(status.snapshotCount + 1),
                      stage.degraded ? " *" : "");
        lv_label_set_text(_hint, text);
    } else if (status.state == State::Paused) {
        lv_label_set_text(_hint, "A: resume   Hold B: finish\nHold A+B: home");
    } else if (status.state == State::Recording) {
        std::snprintf(text, sizeof(text), "A: pause   B: snapshot (%u/32)\nHold A+B: home",
                      static_cast<unsigned>(status.snapshotCount));
        lv_label_set_text(_hint, text);
    } else if (status.state == State::Calibrating) {
        lv_label_set_text(_hint, "Keep device still\nHold A+B: home");
    } else {
        lv_label_set_text(_hint, "A: start   Hold A+B: home");
    }
    const bool show_graphs = !setup && status.state != State::Calibrating;
    for (size_t row = 0; row < _graphs.size(); ++row) {
        Graph& graph = _graphs[row];
        if (!show_graphs) {
            lv_obj_add_flag(graph.object, LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_remove_flag(graph.object, LV_OBJ_FLAG_HIDDEN);
        const auto& range = stage.ranges[graph.axis];
        if (range.valid) {
            if (!finished && (status.validMask & (1U << graph.axis)) != 0) {
                std::snprintf(text, sizeof(text), "%s %.1f  min %.1f  max %.1f deg", axis_names[graph.axis],
                              static_cast<double>(status.angles[graph.axis]), static_cast<double>(range.minimum),
                              static_cast<double>(range.maximum));
            } else {
                std::snprintf(text, sizeof(text), "%s  min %.1f  max %.1f deg", axis_names[graph.axis],
                              static_cast<double>(range.minimum), static_cast<double>(range.maximum));
            }
        } else {
            std::snprintf(text, sizeof(text), "%s  No valid data", axis_names[graph.axis]);
        }
        lv_label_set_text(graph.label, text);
        lv_obj_set_style_text_color(graph.label, lv_color_hex(colors[graph.axis]), 0);
        const Graph& prepared = _prepared_graphs[row];
        graph.points = prepared.points;
        graph.count = prepared.count;
        graph.minimum = prepared.minimum;
        graph.maximum = prepared.maximum;
        graph.elapsedMs = prepared.elapsedMs;
        lv_obj_invalidate(graph.object);
    }
}

void AngleRecorderView::prepareGraph(Graph& graph, const model::angle::Stage& stage,
                                     const model::angle::Sample* samples)
{
    graph.count = 0;
    graph.elapsedMs = stage.elapsedMs;
    const auto& range = stage.ranges[graph.axis];
    const float padding = range.valid ? std::max(1.0f, (range.maximum - range.minimum) * 0.1f) : 1.0f;
    graph.minimum = range.valid ? range.minimum - padding : -1.0f;
    graph.maximum = range.valid ? range.maximum + padding : 1.0f;
    if (samples == nullptr || stage.sampleCount == 0) {
        return;
    }
    uint32_t segment = 0;
    bool valid_previous = false;
    size_t bucket = 0;
    std::array<Point, 4> candidates{};
    bool occupied = false;
    const auto flush = [&]() {
        if (!occupied) {
            return;
        }
        std::sort(candidates.begin(), candidates.end(), [](const Point& left, const Point& right) {
            return left.timeMs < right.timeMs || (left.timeMs == right.timeMs && left.segment < right.segment);
        });
        for (const Point& point : candidates) {
            if (graph.count < graph.points.size()) {
                graph.points[graph.count++] = point;
            }
        }
    };
    for (size_t index = 0; index < stage.sampleCount; ++index) {
        const auto& sample = samples[index];
        const bool valid = (sample.validMask & (1U << graph.axis)) != 0 &&
                           std::isfinite(sample.angles[graph.axis]);
        if (sample.breakBefore || !valid || !valid_previous) {
            ++segment;
        }
        valid_previous = valid;
        if (!valid) {
            continue;
        }
        const size_t next_bucket = static_cast<size_t>(std::min<uint64_t>(127,
            static_cast<uint64_t>(sample.timeMs) * 128 / std::max<uint32_t>(1, stage.elapsedMs)));
        Point point{sample.timeMs, sample.angles[graph.axis], segment};
        if (!occupied || next_bucket != bucket) {
            flush();
            candidates.fill(point);
            occupied = true;
            bucket = next_bucket;
        } else {
            candidates[3] = point;
            if (point.value < candidates[1].value) {
                candidates[1] = point;
            }
            if (point.value > candidates[2].value) {
                candidates[2] = point;
            }
        }
    }
    flush();
}

void AngleRecorderView::onDraw(lv_event_t* event)
{
    auto* graph = static_cast<Graph*>(lv_event_get_user_data(event));
    graph->owner->drawGraph(*graph, lv_event_get_layer(event));
}

void AngleRecorderView::drawGraph(Graph& graph, lv_layer_t* layer)
{
    lv_area_t area;
    lv_obj_get_coords(graph.object, &area);
    const int32_t left = area.x1 + 32;
    const int32_t right = area.x2 - 6;
    const int32_t top = area.y1 + 25;
    const int32_t bottom = area.y2 - 18;
    lv_draw_line_dsc_t line;
    lv_draw_line_dsc_init(&line);
    line.width = 1;
    line.color = lv_color_hex(0x344658);
    const auto draw_line = [&](float x1, float y1, float x2, float y2) {
        line.p1.x = x1;
        line.p1.y = y1;
        line.p2.x = x2;
        line.p2.y = y2;
        lv_draw_line(layer, &line);
    };
    draw_line(static_cast<float>(left), static_cast<float>(top), static_cast<float>(left), static_cast<float>(bottom));
    for (int row = 0; row <= 2; ++row) {
        const float y = static_cast<float>(top + (bottom - top) * row / 2);
        draw_line(static_cast<float>(left), y, static_cast<float>(right), y);
    }
    lv_draw_label_dsc_t label;
    lv_draw_label_dsc_init(&label);
    label.color = lv_color_hex(0x9AAEC4);
    label.font = &lv_font_montserrat_10;
    char text[32];
    const auto draw_text = [&](const char* value, int32_t x, int32_t y, int32_t width) {
        label.text = value;
        label.text_local = 1;
        lv_area_t bounds{x, y, x + width, y + 13};
        lv_draw_label(layer, &label, &bounds);
    };
    std::snprintf(text, sizeof(text), "%.0f", static_cast<double>(graph.maximum));
    draw_text(text, area.x1, top, 30);
    std::snprintf(text, sizeof(text), "%.0f", static_cast<double>(graph.minimum));
    draw_text(text, area.x1, bottom - 10, 30);
    draw_text("0 s", left, bottom + 3, 30);
    std::snprintf(text, sizeof(text), "%.1f s", static_cast<double>(graph.elapsedMs) / 1000.0);
    draw_text(text, right - 65, bottom + 3, 65);
    line.color = lv_color_hex(colors[graph.axis]);
    line.width = 2;
    const auto position = [&](const Point& point) {
        return lv_point_precise_t{
            static_cast<lv_value_precise_t>(static_cast<float>(left) + static_cast<float>(right - left) *
                static_cast<float>(point.timeMs) / static_cast<float>(std::max<uint32_t>(1, graph.elapsedMs))),
            static_cast<lv_value_precise_t>(static_cast<float>(bottom) - static_cast<float>(bottom - top) *
                (point.value - graph.minimum) / (graph.maximum - graph.minimum))};
    };
    for (size_t index = 0; index < graph.count; ++index) {
        const auto current = position(graph.points[index]);
        if (index > 0 && graph.points[index - 1].segment == graph.points[index].segment) {
            const auto previous = position(graph.points[index - 1]);
            draw_line(previous.x, previous.y, current.x, current.y);
        } else {
            draw_line(current.x, current.y, current.x + 1.0f, current.y);
        }
    }
}

}  // namespace view
