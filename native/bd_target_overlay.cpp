// Minimal in-place RGB aim overlay for the cell-phone tracking station.
// It consumes the private "cell_phone_aim" HailoLandmarks object, draws the
// active-target vector, and removes that object before hailooverlay sees it.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gst/gst.h>
#include <gst/video/gstvideofilter.h>
#include <hailo/tappas/gst_hailo_meta.hpp>

typedef struct _GstBdTargetOverlay {
    GstVideoFilter parent;
} GstBdTargetOverlay;

typedef struct _GstBdTargetOverlayClass {
    GstVideoFilterClass parent_class;
} GstBdTargetOverlayClass;

G_DEFINE_TYPE(GstBdTargetOverlay, gst_bd_target_overlay, GST_TYPE_VIDEO_FILTER)

constexpr gint LOCK_TOLERANCE_PX = 25;

static inline void set_rgb(
    guint8 *data, gint stride, gint width, gint height,
    gint x, gint y, guint8 red, guint8 green, guint8 blue)
{
    if (x < 0 || y < 0 || x >= width || y >= height) {
        return;
    }
    guint8 *pixel = data + y * stride + x * 3;
    pixel[0] = red;
    pixel[1] = green;
    pixel[2] = blue;
}

static void draw_red_line(
    guint8 *data, gint stride, gint width, gint height,
    gint x0, gint y0, gint x1, gint y1)
{
    gint dx = std::abs(x1 - x0);
    gint step_x = x0 < x1 ? 1 : -1;
    gint dy = -std::abs(y1 - y0);
    gint step_y = y0 < y1 ? 1 : -1;
    gint error = dx + dy;

    while (true) {
        set_rgb(data, stride, width, height, x0, y0, 255, 0, 0);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const gint doubled = 2 * error;
        if (doubled >= dy) {
            error += dy;
            x0 += step_x;
        }
        if (doubled <= dx) {
            error += dx;
            y0 += step_y;
        }
    }
}

static void draw_lock_tolerance_box(
    guint8 *data, gint stride, gint width, gint height,
    gint center_x, gint center_y)
{
    const gint left = center_x - LOCK_TOLERANCE_PX;
    const gint right = center_x + LOCK_TOLERANCE_PX;
    const gint top = center_y - LOCK_TOLERANCE_PX;
    const gint bottom = center_y + LOCK_TOLERANCE_PX;

    // One-pixel yellow outline: the interior remains unobstructed.
    for (gint x = left; x <= right; ++x) {
        set_rgb(data, stride, width, height, x, top, 255, 255, 0);
        set_rgb(data, stride, width, height, x, bottom, 255, 255, 0);
    }
    for (gint y = top; y <= bottom; ++y) {
        set_rgb(data, stride, width, height, left, y, 255, 255, 0);
        set_rgb(data, stride, width, height, right, y, 255, 255, 0);
    }
}

static GstFlowReturn gst_bd_target_overlay_transform_frame_ip(
    GstVideoFilter *, GstVideoFrame *frame)
{
    guint8 *data = static_cast<guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(frame, 0));
    const gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
    const gint width = GST_VIDEO_FRAME_WIDTH(frame);
    const gint height = GST_VIDEO_FRAME_HEIGHT(frame);

    HailoROIPtr roi = get_hailo_main_roi(frame->buffer, false);
    if (roi) {
        const auto landmarks_objects = roi->get_objects_typed(HAILO_LANDMARKS);
        for (const auto &object : landmarks_objects) {
            auto landmarks = std::dynamic_pointer_cast<HailoLandmarks>(object);
            if (!landmarks || landmarks->get_landmarks_type() != "cell_phone_aim") {
                continue;
            }
            const auto points = landmarks->get_points();
            if (points.size() >= 2) {
                const gint x0 = std::clamp(
                    static_cast<gint>(std::lround(points[0].x() * (width - 1))),
                    0, width - 1);
                const gint y0 = std::clamp(
                    static_cast<gint>(std::lround(points[0].y() * (height - 1))),
                    0, height - 1);
                const gint x1 = std::clamp(
                    static_cast<gint>(std::lround(points[1].x() * (width - 1))),
                    0, width - 1);
                const gint y1 = std::clamp(
                    static_cast<gint>(std::lround(points[1].y() * (height - 1))),
                    0, height - 1);
                draw_red_line(data, stride, width, height, x0, y0, x1, y1);
                set_rgb(data, stride, width, height, x1, y1, 255, 0, 0);
            }
            roi->remove_object(object);
        }
    }

    // The lock condition is abs(error_x/y) <= 25 px. Draw that exact area
    // around the optical centre, then keep the tiny white centre marker.
    const gint center_x = width / 2;
    const gint center_y = height / 2;
    draw_lock_tolerance_box(
        data, stride, width, height, center_x, center_y);
    for (gint offset = -2; offset <= 2; ++offset) {
        set_rgb(data, stride, width, height,
                center_x + offset, center_y + offset, 255, 255, 255);
        set_rgb(data, stride, width, height,
                center_x + offset, center_y - offset, 255, 255, 255);
    }
    return GST_FLOW_OK;
}

static void gst_bd_target_overlay_class_init(GstBdTargetOverlayClass *klass)
{
    GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
    GstVideoFilterClass *video_filter_class = GST_VIDEO_FILTER_CLASS(klass);

    gst_element_class_set_static_metadata(
        element_class,
        "Cell-phone target overlay",
        "Filter/Effect/Video",
        "Draws an in-place active-target vector on RGB frames",
        "Cell Phone Tracking Control UI");

    GstCaps *caps = gst_caps_from_string("video/x-raw,format=RGB");
    gst_element_class_add_pad_template(
        element_class,
        gst_pad_template_new("sink", GST_PAD_SINK, GST_PAD_ALWAYS, caps));
    gst_element_class_add_pad_template(
        element_class,
        gst_pad_template_new("src", GST_PAD_SRC, GST_PAD_ALWAYS, caps));
    gst_caps_unref(caps);

    video_filter_class->transform_frame_ip =
        gst_bd_target_overlay_transform_frame_ip;
}

static void gst_bd_target_overlay_init(GstBdTargetOverlay *) {}

static gboolean plugin_init(GstPlugin *plugin)
{
    return gst_element_register(
        plugin, "bdtargetoverlay", GST_RANK_NONE,
        gst_bd_target_overlay_get_type());
}

#ifndef PACKAGE
#define PACKAGE "bird-drone-control-ui"
#endif

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    bdtargetoverlay,
    "Cell-phone in-place RGB target overlay",
    plugin_init,
    "1.0.0",
    "LGPL",
    PACKAGE,
    "local")
