#include "perception/streaming/rtsp_server.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#if defined(PERCEPTION_HAS_GSTREAMER)
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#endif

#if defined(PERCEPTION_HAS_SPDLOG)
#include <spdlog/spdlog.h>
#endif

namespace perception::streaming {

class RtspServer::Impl {
public:
    explicit Impl(core::StreamingConfig value) : config(std::move(value)) {}

    core::StreamingConfig config;
    bool running{false};

#if defined(PERCEPTION_HAS_GSTREAMER)
    struct MountContext {
        bool enabled{false};
        std::string path;
        std::mutex mutex;
        GstElement* appsrc{nullptr};
        std::uint64_t first_timestamp_ns{0};
    };

    GMainLoop* main_loop{nullptr};
    GstRTSPServer* server{nullptr};
    guint server_source_id{0};
    std::thread main_loop_thread;
    std::map<StreamId, std::unique_ptr<MountContext>> mounts;

    static void media_configure(GstRTSPMediaFactory*, GstRTSPMedia* media, gpointer user_data) {
        auto* mount = static_cast<MountContext*>(user_data);
        GstElement* pipeline = gst_rtsp_media_get_element(media);
        GstElement* source = gst_bin_get_by_name_recurse_up(GST_BIN(pipeline), "source");
        gst_object_unref(pipeline);
        if (source == nullptr) {
            return;
        }

        g_object_set(source, "is-live", TRUE, "format", GST_FORMAT_TIME, "block", FALSE,
                     "max-buffers", static_cast<guint64>(2), nullptr);
        {
            std::scoped_lock lock(mount->mutex);
            if (mount->appsrc != nullptr) {
                gst_object_unref(mount->appsrc);
            }
            mount->appsrc = source;
            mount->first_timestamp_ns = 0;
        }
    }

    [[nodiscard]] auto encoder_pipeline() const -> std::string {
        GstElementFactory* hardware_factory = gst_element_factory_find("nvv4l2h264enc");
        const bool hardware_available = hardware_factory != nullptr;
        if (hardware_factory != nullptr) {
            gst_object_unref(hardware_factory);
        }

        std::ostringstream pipeline;
        pipeline << "appsrc name=source ! queue max-size-buffers=" << config.queue_capacity
                 << " leaky=downstream ! videoconvert ! video/x-raw,format=I420 ! ";

        if (config.hardware_encoder && hardware_available) {
            pipeline << "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
                     << "nvv4l2h264enc maxperf-enable=true control-rate=1 bitrate="
                     << config.bitrate_kbps * 1'000U
                     << " iframeinterval=" << config.keyframe_interval
                     << " idrinterval=" << config.keyframe_interval
                     << " insert-sps-pps=true ! ";
#if defined(PERCEPTION_HAS_SPDLOG)
            spdlog::info("RTSP using Jetson hardware H.264 encoder");
#endif
        } else {
            if (config.hardware_encoder && !config.allow_software_fallback) {
                return {};
            }
            pipeline << "x264enc tune=zerolatency speed-preset=ultrafast bitrate="
                     << config.bitrate_kbps << " key-int-max=" << config.keyframe_interval
                     << " bframes=0 sliced-threads=true ! ";
#if defined(PERCEPTION_HAS_SPDLOG)
            spdlog::warn("Jetson H.264 encoder unavailable; using x264 low-latency fallback");
#endif
        }

        pipeline << "h264parse config-interval=-1 ! "
                 << "rtph264pay name=pay0 pt=96 config-interval=1 aggregate-mode=none";
        return pipeline.str();
    }

    bool add_mount(GstRTSPMountPoints* mount_points, StreamId id,
                   const core::RtspStreamConfig& stream, const std::string& launch) {
        auto context = std::make_unique<MountContext>();
        context->enabled = stream.enabled;
        context->path = stream.path;
        MountContext* context_ptr = context.get();
        mounts.emplace(id, std::move(context));
        if (!stream.enabled) {
            return true;
        }
        if (stream.path.empty() || stream.path.front() != '/') {
            return false;
        }
        GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
        gst_rtsp_media_factory_set_shared(factory, TRUE);
        gst_rtsp_media_factory_set_suspend_mode(factory, GST_RTSP_SUSPEND_MODE_NONE);
        gst_rtsp_media_factory_set_launch(factory, launch.c_str());
        g_signal_connect(factory, "media-configure", G_CALLBACK(media_configure), context_ptr);
        gst_rtsp_mount_points_add_factory(mount_points, stream.path.c_str(), factory);
        return true;
    }

    void clear_sources() noexcept {
        for (auto& entry : mounts) {
            auto& mount = entry.second;
            std::scoped_lock lock(mount->mutex);
            if (mount->appsrc != nullptr) {
                gst_object_unref(mount->appsrc);
                mount->appsrc = nullptr;
            }
            mount->first_timestamp_ns = 0;
        }
    }
#endif
};

RtspServer::RtspServer(core::StreamingConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}
RtspServer::~RtspServer() { stop(); }
RtspServer::RtspServer(RtspServer&&) noexcept = default;
auto RtspServer::operator=(RtspServer&&) noexcept -> RtspServer& = default;

bool RtspServer::start() {
#if defined(PERCEPTION_HAS_GSTREAMER)
    if (impl_->running) {
        return true;
    }
    gst_init(nullptr, nullptr);
    const std::string launch = impl_->encoder_pipeline();
    if (launch.empty()) {
        return false;
    }

    impl_->main_loop = g_main_loop_new(nullptr, FALSE);
    impl_->server = gst_rtsp_server_new();
    gst_rtsp_server_set_address(impl_->server, impl_->config.bind_address.c_str());
    const std::string port = std::to_string(impl_->config.port);
    gst_rtsp_server_set_service(impl_->server, port.c_str());

    GstRTSPMountPoints* mount_points = gst_rtsp_server_get_mount_points(impl_->server);
    const bool configured =
        impl_->add_mount(mount_points, StreamId::Rgb, impl_->config.rgb, launch) &&
        impl_->add_mount(mount_points, StreamId::InfraredLeft, impl_->config.infrared_left,
                         launch) &&
        impl_->add_mount(mount_points, StreamId::InfraredRight, impl_->config.infrared_right,
                         launch) &&
        impl_->add_mount(mount_points, StreamId::DepthVisual, impl_->config.depth_visual, launch);
    gst_object_unref(mount_points);
    if (!configured) {
        stop();
        return false;
    }

    impl_->server_source_id = gst_rtsp_server_attach(impl_->server, nullptr);
    if (impl_->server_source_id == 0) {
        stop();
        return false;
    }
    impl_->running = true;
    impl_->main_loop_thread = std::thread([this] { g_main_loop_run(impl_->main_loop); });
    return true;
#else
    return false;
#endif
}

void RtspServer::stop() noexcept {
#if defined(PERCEPTION_HAS_GSTREAMER)
    if (impl_->main_loop != nullptr) {
        g_main_loop_quit(impl_->main_loop);
    }
    if (impl_->main_loop_thread.joinable()) {
        impl_->main_loop_thread.join();
    }
    if (impl_->server_source_id != 0) {
        g_source_remove(impl_->server_source_id);
        impl_->server_source_id = 0;
    }
    impl_->clear_sources();
    impl_->mounts.clear();
    if (impl_->server != nullptr) {
        g_object_unref(impl_->server);
        impl_->server = nullptr;
    }
    if (impl_->main_loop != nullptr) {
        g_main_loop_unref(impl_->main_loop);
        impl_->main_loop = nullptr;
    }
#endif
    impl_->running = false;
}

bool RtspServer::publish(StreamId stream, const camera::ImageFrame& frame) {
#if defined(PERCEPTION_HAS_GSTREAMER)
    const auto mount_it = impl_->mounts.find(stream);
    if (mount_it == impl_->mounts.end() || !mount_it->second->enabled) {
        return true;
    }
    if (!impl_->running || frame.data.empty() || frame.width <= 0 || frame.height <= 0) {
        return false;
    }

    auto& mount = *mount_it->second;
    GstElement* source = nullptr;
    std::uint64_t pts_ns = 0;
    {
        std::scoped_lock lock(mount.mutex);
        if (mount.appsrc == nullptr) {
            return true;
        }
        source = GST_ELEMENT(gst_object_ref(mount.appsrc));
        if (mount.first_timestamp_ns == 0) {
            mount.first_timestamp_ns = frame.capture_timestamp_ns;
        }
        if (frame.capture_timestamp_ns >= mount.first_timestamp_ns) {
            pts_ns = frame.capture_timestamp_ns - mount.first_timestamp_ns;
        }
    }

    const char* format = frame.format == camera::PixelFormat::Gray8 ? "GRAY8" : "BGR";
    GstCaps* caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, format, "width",
                                        G_TYPE_INT, frame.width, "height", G_TYPE_INT, frame.height,
                                        "framerate", GST_TYPE_FRACTION, 30, 1, nullptr);
    gst_app_src_set_caps(GST_APP_SRC(source), caps);
    gst_caps_unref(caps);

    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, frame.data.size(), nullptr);
    if (buffer == nullptr) {
        gst_object_unref(source);
        return false;
    }
    gst_buffer_fill(buffer, 0, frame.data.data(), frame.data.size());
    GST_BUFFER_PTS(buffer) = static_cast<GstClockTime>(pts_ns);
    GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DURATION(buffer) = GST_SECOND / 30U;
    const GstFlowReturn result = gst_app_src_push_buffer(GST_APP_SRC(source), buffer);
    gst_object_unref(source);
    return result == GST_FLOW_OK || result == GST_FLOW_FLUSHING;
#else
    (void)stream;
    (void)frame;
    return false;
#endif
}

}  // namespace perception::streaming
