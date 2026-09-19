#include "perception/dataset/dataset.hpp"

#include "perception/transport/json.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iterator>
#include <thread>
#include <utility>

namespace perception::dataset {
namespace {

namespace json = transport::json;

constexpr const char* kManifestName = "manifest.json";
constexpr const char* kIndexName = "index.jsonl";
constexpr const char* kBlobName = "blob.bin";

auto pixel_format_name(camera::PixelFormat format) -> const char* {
    switch (format) {
    case camera::PixelFormat::Gray8:
        return "gray8";
    case camera::PixelFormat::DepthZ16:
        return "z16";
    case camera::PixelFormat::Bgr8:
        break;
    }
    return "bgr8";
}

auto pixel_format_from_name(const std::string& name) -> camera::PixelFormat {
    if (name == "gray8") {
        return camera::PixelFormat::Gray8;
    }
    if (name == "z16") {
        return camera::PixelFormat::DepthZ16;
    }
    return camera::PixelFormat::Bgr8;
}

auto integer(std::uint64_t value) -> json::Value {
    return json::Value::integer(static_cast<std::int64_t>(value));
}

auto read_unsigned(const json::Value& node, std::string_view key) -> std::uint64_t {
    const json::Value* member = node.find(key);
    if (member == nullptr) {
        return 0;
    }
    const auto number = member->as_number();
    return (!number || *number < 0.0) ? 0U : static_cast<std::uint64_t>(*number);
}

auto read_int(const json::Value& node, std::string_view key) -> int {
    const json::Value* member = node.find(key);
    if (member == nullptr) {
        return 0;
    }
    const auto number = member->as_number();
    return number ? static_cast<int>(*number) : 0;
}

auto read_double(const json::Value& node, std::string_view key) -> double {
    const json::Value* member = node.find(key);
    if (member == nullptr) {
        return 0.0;
    }
    return member->as_number().value_or(0.0);
}

auto read_string(const json::Value& node, std::string_view key) -> std::string {
    const json::Value* member = node.find(key);
    if (member == nullptr) {
        return {};
    }
    const std::string* text = member->as_string();
    return text == nullptr ? std::string{} : *text;
}

auto intrinsics_to_json(const camera::CameraIntrinsics& intrinsics) -> json::Value {
    json::Value node;
    node.set("width", json::Value::integer(intrinsics.width));
    node.set("height", json::Value::integer(intrinsics.height));
    node.set("principal_x", json::Value::real(static_cast<double>(intrinsics.principal_x)));
    node.set("principal_y", json::Value::real(static_cast<double>(intrinsics.principal_y)));
    node.set("focal_x", json::Value::real(static_cast<double>(intrinsics.focal_x)));
    node.set("focal_y", json::Value::real(static_cast<double>(intrinsics.focal_y)));
    node.set("distortion_model", json::Value::string(intrinsics.distortion_model));
    json::ArrayItems coefficients;
    for (const float coefficient : intrinsics.coefficients) {
        coefficients.push_back(json::Value::real(static_cast<double>(coefficient)));
    }
    node.set("coefficients", json::Value::array(std::move(coefficients)));
    return node;
}

auto intrinsics_from_json(const json::Value& node) -> camera::CameraIntrinsics {
    camera::CameraIntrinsics intrinsics;
    intrinsics.width = read_int(node, "width");
    intrinsics.height = read_int(node, "height");
    intrinsics.principal_x = static_cast<float>(read_double(node, "principal_x"));
    intrinsics.principal_y = static_cast<float>(read_double(node, "principal_y"));
    intrinsics.focal_x = static_cast<float>(read_double(node, "focal_x"));
    intrinsics.focal_y = static_cast<float>(read_double(node, "focal_y"));
    intrinsics.distortion_model = read_string(node, "distortion_model");
    if (const json::Value* coefficients = node.find("coefficients"); coefficients != nullptr) {
        if (const json::ArrayItems* items = coefficients->as_array(); items != nullptr) {
            for (std::size_t index = 0; index < items->size() && index < intrinsics.coefficients.size();
                 ++index) {
                intrinsics.coefficients[index] =
                    static_cast<float>((*items)[index].as_number().value_or(0.0));
            }
        }
    }
    return intrinsics;
}

auto manifest_to_json(const DatasetManifest& manifest) -> json::Value {
    json::Value root;
    root.set("format_version", json::Value::integer(manifest.format_version));
    root.set("created_utc", json::Value::string(manifest.created_utc));

    json::Value device;
    device.set("model", json::Value::string(manifest.device.model));
    device.set("serial", json::Value::string(manifest.device.serial));
    device.set("firmware", json::Value::string(manifest.device.firmware));
    device.set("usb_mode", json::Value::string(manifest.device.usb_mode));
    device.set("depth_scale_m", json::Value::real(static_cast<double>(manifest.device.depth_scale_m)));
    root.set("device", std::move(device));

    if (manifest.rgb_intrinsics.has_value()) {
        root.set("rgb_intrinsics", intrinsics_to_json(*manifest.rgb_intrinsics));
    }
    if (manifest.depth_intrinsics.has_value()) {
        root.set("depth_intrinsics", intrinsics_to_json(*manifest.depth_intrinsics));
    }

    json::Value mount;
    mount.set("x_m", json::Value::real(manifest.mount.x_m));
    mount.set("y_m", json::Value::real(manifest.mount.y_m));
    mount.set("z_m", json::Value::real(manifest.mount.z_m));
    mount.set("roll_deg", json::Value::real(manifest.mount.roll_deg));
    mount.set("pitch_deg", json::Value::real(manifest.mount.pitch_deg));
    mount.set("yaw_deg", json::Value::real(manifest.mount.yaw_deg));
    root.set("mount", std::move(mount));
    root.set("notes", json::Value::string(manifest.notes));
    return root;
}

auto manifest_from_json(const json::Value& root, DatasetManifest& manifest, std::string& error)
    -> bool {
    manifest.format_version = read_int(root, "format_version");
    if (manifest.format_version != kFormatVersion) {
        error = "unsupported dataset format_version " + std::to_string(manifest.format_version);
        return false;
    }
    manifest.created_utc = read_string(root, "created_utc");
    if (const json::Value* device = root.find("device"); device != nullptr) {
        manifest.device.model = read_string(*device, "model");
        manifest.device.serial = read_string(*device, "serial");
        manifest.device.firmware = read_string(*device, "firmware");
        manifest.device.usb_mode = read_string(*device, "usb_mode");
        manifest.device.depth_scale_m = static_cast<float>(read_double(*device, "depth_scale_m"));
    }
    if (const json::Value* rgb = root.find("rgb_intrinsics"); rgb != nullptr) {
        manifest.rgb_intrinsics = intrinsics_from_json(*rgb);
    }
    if (const json::Value* depth = root.find("depth_intrinsics"); depth != nullptr) {
        manifest.depth_intrinsics = intrinsics_from_json(*depth);
    }
    if (const json::Value* mount = root.find("mount"); mount != nullptr) {
        manifest.mount.x_m = read_double(*mount, "x_m");
        manifest.mount.y_m = read_double(*mount, "y_m");
        manifest.mount.z_m = read_double(*mount, "z_m");
        manifest.mount.roll_deg = read_double(*mount, "roll_deg");
        manifest.mount.pitch_deg = read_double(*mount, "pitch_deg");
        manifest.mount.yaw_deg = read_double(*mount, "yaw_deg");
    }
    manifest.notes = read_string(root, "notes");
    return true;
}

auto image_to_json(const camera::ImageFrame& image, std::uint64_t offset) -> json::Value {
    json::Value node;
    node.set("sensor_timestamp_ns", integer(image.sensor_timestamp_ns));
    node.set("capture_timestamp_ns", integer(image.capture_timestamp_ns));
    node.set("frame_number", integer(image.frame_number));
    node.set("width", json::Value::integer(image.width));
    node.set("height", json::Value::integer(image.height));
    node.set("stride_bytes", json::Value::integer(image.stride_bytes));
    node.set("format", json::Value::string(pixel_format_name(image.format)));
    node.set("offset", integer(offset));
    node.set("bytes", integer(image.data.size()));
    return node;
}

auto depth_to_json(const camera::DepthFrame& depth, std::uint64_t offset) -> json::Value {
    json::Value node;
    node.set("sensor_timestamp_ns", integer(depth.sensor_timestamp_ns));
    node.set("capture_timestamp_ns", integer(depth.capture_timestamp_ns));
    node.set("frame_number", integer(depth.frame_number));
    node.set("width", json::Value::integer(depth.width));
    node.set("height", json::Value::integer(depth.height));
    node.set("depth_scale_m", json::Value::real(static_cast<double>(depth.depth_scale_m)));
    node.set("offset", integer(offset));
    node.set("bytes", integer(depth.data.size() * sizeof(std::uint16_t)));
    return node;
}

auto imu_to_json(const camera::ImuBatch& imu) -> json::Value {
    json::ArrayItems accelerometer;
    accelerometer.reserve(imu.accelerometer.size());
    for (const auto& sample : imu.accelerometer) {
        json::Value entry;
        entry.set("sensor_timestamp_ns", integer(sample.sensor_timestamp_ns));
        entry.set("capture_timestamp_ns", integer(sample.capture_timestamp_ns));
        entry.set("x", json::Value::real(static_cast<double>(sample.acceleration_mps2[0])));
        entry.set("y", json::Value::real(static_cast<double>(sample.acceleration_mps2[1])));
        entry.set("z", json::Value::real(static_cast<double>(sample.acceleration_mps2[2])));
        accelerometer.push_back(std::move(entry));
    }
    json::ArrayItems gyroscope;
    gyroscope.reserve(imu.gyroscope.size());
    for (const auto& sample : imu.gyroscope) {
        json::Value entry;
        entry.set("sensor_timestamp_ns", integer(sample.sensor_timestamp_ns));
        entry.set("capture_timestamp_ns", integer(sample.capture_timestamp_ns));
        entry.set("x", json::Value::real(static_cast<double>(sample.angular_velocity_rps[0])));
        entry.set("y", json::Value::real(static_cast<double>(sample.angular_velocity_rps[1])));
        entry.set("z", json::Value::real(static_cast<double>(sample.angular_velocity_rps[2])));
        gyroscope.push_back(std::move(entry));
    }

    json::Value node;
    node.set("accelerometer", json::Value::array(std::move(accelerometer)));
    node.set("gyroscope", json::Value::array(std::move(gyroscope)));
    node.set("accelerometer_dropped", integer(imu.accelerometer_dropped));
    node.set("gyroscope_dropped", integer(imu.gyroscope_dropped));
    return node;
}

void imu_from_json(const json::Value& node, camera::ImuBatch& imu) {
    if (const json::Value* accelerometer = node.find("accelerometer"); accelerometer != nullptr) {
        if (const json::ArrayItems* items = accelerometer->as_array(); items != nullptr) {
            imu.accelerometer.reserve(items->size());
            for (const json::Value& entry : *items) {
                camera::AccelerometerSample sample;
                sample.sensor_timestamp_ns = read_unsigned(entry, "sensor_timestamp_ns");
                sample.capture_timestamp_ns = read_unsigned(entry, "capture_timestamp_ns");
                sample.acceleration_mps2 = {static_cast<float>(read_double(entry, "x")),
                                            static_cast<float>(read_double(entry, "y")),
                                            static_cast<float>(read_double(entry, "z"))};
                imu.accelerometer.push_back(sample);
            }
        }
    }
    if (const json::Value* gyroscope = node.find("gyroscope"); gyroscope != nullptr) {
        if (const json::ArrayItems* items = gyroscope->as_array(); items != nullptr) {
            imu.gyroscope.reserve(items->size());
            for (const json::Value& entry : *items) {
                camera::GyroscopeSample sample;
                sample.sensor_timestamp_ns = read_unsigned(entry, "sensor_timestamp_ns");
                sample.capture_timestamp_ns = read_unsigned(entry, "capture_timestamp_ns");
                sample.angular_velocity_rps = {static_cast<float>(read_double(entry, "x")),
                                               static_cast<float>(read_double(entry, "y")),
                                               static_cast<float>(read_double(entry, "z"))};
                imu.gyroscope.push_back(sample);
            }
        }
    }
    imu.accelerometer_dropped = read_unsigned(node, "accelerometer_dropped");
    imu.gyroscope_dropped = read_unsigned(node, "gyroscope_dropped");
}

void image_from_json(const json::Value& node, const std::vector<std::uint8_t>& payload,
                     camera::ImageFrame& image) {
    image.sensor_timestamp_ns = read_unsigned(node, "sensor_timestamp_ns");
    image.capture_timestamp_ns = read_unsigned(node, "capture_timestamp_ns");
    image.frame_number = read_unsigned(node, "frame_number");
    image.width = read_int(node, "width");
    image.height = read_int(node, "height");
    image.stride_bytes = read_int(node, "stride_bytes");
    image.format = pixel_format_from_name(read_string(node, "format"));
    image.data = payload;
}

auto equals_ignoring_case(std::string_view lhs, std::string_view rhs) -> bool {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t index = 0; index < lhs.size(); ++index) {
        const auto left = static_cast<unsigned char>(lhs[index]);
        const auto right = static_cast<unsigned char>(rhs[index]);
        if (std::tolower(left) != std::tolower(right)) {
            return false;
        }
    }
    return true;
}

} // namespace

auto select_stream_intrinsics(const camera::CameraDeviceInfo& device, std::string_view stream,
                              int width, int height, bool* exact_resolution)
    -> std::optional<camera::CameraIntrinsics> {
    if (exact_resolution != nullptr) {
        *exact_resolution = false;
    }
    for (const auto& profile : device.stream_profiles) {
        if (equals_ignoring_case(profile.stream, stream) && profile.width == width &&
            profile.height == height && profile.intrinsics.has_value()) {
            if (exact_resolution != nullptr) {
                *exact_resolution = true;
            }
            return profile.intrinsics;
        }
    }
    // Fall back to any profile of that stream so the recording still carries something usable for
    // inspection, but the caller is told the resolution did not match.
    for (const auto& profile : device.stream_profiles) {
        if (equals_ignoring_case(profile.stream, stream) && profile.intrinsics.has_value()) {
            return profile.intrinsics;
        }
    }
    return std::nullopt;
}

// -------------------------------------------------------------------------------------------
// Writer
// -------------------------------------------------------------------------------------------

class DatasetWriter::Impl {
  public:
    std::ofstream index;
    std::ofstream blob;
    std::uint64_t offset{0};
    std::uint64_t frames{0};
    bool open{false};

    auto write_payload(const void* data, std::size_t bytes, std::uint64_t& payload_offset,
                       std::string& error) -> bool {
        payload_offset = offset;
        if (bytes == 0) {
            return true;
        }
        blob.write(static_cast<const char*>(data), static_cast<std::streamsize>(bytes));
        if (!blob) {
            error = "failed to write dataset payload";
            return false;
        }
        offset += bytes;
        return true;
    }
};

DatasetWriter::DatasetWriter() : impl_(std::make_unique<Impl>()) {}

DatasetWriter::~DatasetWriter() {
    std::string ignored;
    (void)close(ignored);
}

auto DatasetWriter::open(const std::filesystem::path& directory, const DatasetManifest& manifest,
                         std::string& error) -> bool {
    error.clear();
    std::error_code filesystem_error;
    if (std::filesystem::exists(directory / kIndexName, filesystem_error)) {
        // Recordings are field evidence; refusing to overwrite is deliberate.
        error = "dataset already exists at " + directory.string();
        return false;
    }
    std::filesystem::create_directories(directory, filesystem_error);
    if (filesystem_error) {
        error = "could not create " + directory.string() + ": " + filesystem_error.message();
        return false;
    }

    std::ofstream manifest_file(directory / kManifestName, std::ios::binary | std::ios::trunc);
    if (!manifest_file) {
        error = "could not write " + std::string(kManifestName);
        return false;
    }
    manifest_file << manifest_to_json(manifest).dump() << '\n';
    if (!manifest_file) {
        error = "failed while writing " + std::string(kManifestName);
        return false;
    }

    impl_->index.open(directory / kIndexName, std::ios::binary | std::ios::trunc);
    impl_->blob.open(directory / kBlobName, std::ios::binary | std::ios::trunc);
    if (!impl_->index || !impl_->blob) {
        error = "could not open dataset index or blob for writing";
        return false;
    }
    impl_->offset = 0;
    impl_->frames = 0;
    impl_->open = true;
    return true;
}

auto DatasetWriter::append(const camera::CameraFrameSet& frame_set, std::string& error) -> bool {
    error.clear();
    if (!impl_->open) {
        error = "dataset writer is not open";
        return false;
    }

    json::Value entry;
    entry.set("capture_timestamp_ns", integer(frame_set.capture_timestamp_ns));

    const auto write_image = [&](const char* key, const camera::ImageFrame& image) -> bool {
        if (image.data.empty()) {
            return true;
        }
        std::uint64_t payload_offset = 0;
        if (!impl_->write_payload(image.data.data(), image.data.size(), payload_offset, error)) {
            return false;
        }
        entry.set(key, image_to_json(image, payload_offset));
        return true;
    };

    if (!write_image("rgb", frame_set.rgb) || !write_image("ir_left", frame_set.ir_left) ||
        !write_image("ir_right", frame_set.ir_right)) {
        return false;
    }

    if (!frame_set.depth.data.empty()) {
        std::uint64_t payload_offset = 0;
        const std::size_t bytes = frame_set.depth.data.size() * sizeof(std::uint16_t);
        if (!impl_->write_payload(frame_set.depth.data.data(), bytes, payload_offset, error)) {
            return false;
        }
        entry.set("depth", depth_to_json(frame_set.depth, payload_offset));
    }

    if (frame_set.imu.has_value()) {
        entry.set("imu", imu_to_json(*frame_set.imu));
    }

    impl_->index << entry.dump() << '\n';
    if (!impl_->index) {
        error = "failed to write dataset index entry";
        return false;
    }
    ++impl_->frames;
    return true;
}

auto DatasetWriter::close(std::string& error) -> bool {
    if (!impl_->open) {
        return true;
    }
    impl_->index.flush();
    impl_->blob.flush();
    const bool healthy = static_cast<bool>(impl_->index) && static_cast<bool>(impl_->blob);
    impl_->index.close();
    impl_->blob.close();
    impl_->open = false;
    if (!healthy) {
        error = "dataset files reported an error while closing";
        return false;
    }
    return true;
}

auto DatasetWriter::frames_written() const noexcept -> std::uint64_t { return impl_->frames; }

auto DatasetWriter::bytes_written() const noexcept -> std::uint64_t { return impl_->offset; }

// -------------------------------------------------------------------------------------------
// Reader
// -------------------------------------------------------------------------------------------

class DatasetReader::Impl {
  public:
    DatasetManifest manifest;
    std::vector<json::Value> entries;
    mutable std::ifstream blob;

    auto read_payload(std::uint64_t offset, std::uint64_t bytes, std::vector<std::uint8_t>& out,
                      std::string& error) const -> bool {
        out.resize(static_cast<std::size_t>(bytes));
        if (bytes == 0) {
            return true;
        }
        blob.clear();
        blob.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
        blob.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(bytes));
        if (blob.gcount() != static_cast<std::streamsize>(bytes)) {
            error = "dataset blob is shorter than its index claims";
            return false;
        }
        return true;
    }
};

DatasetReader::DatasetReader() : impl_(std::make_unique<Impl>()) {}

DatasetReader::~DatasetReader() = default;

auto DatasetReader::open(const std::filesystem::path& directory, std::string& error) -> bool {
    error.clear();
    std::ifstream manifest_file(directory / kManifestName, std::ios::binary);
    if (!manifest_file) {
        error = "dataset has no " + std::string(kManifestName) + " in " + directory.string();
        return false;
    }
    const std::string manifest_text((std::istreambuf_iterator<char>(manifest_file)),
                                    std::istreambuf_iterator<char>());
    std::string parse_error;
    const auto manifest_json = json::parse(manifest_text, &parse_error);
    if (!manifest_json) {
        error = "malformed manifest: " + parse_error;
        return false;
    }
    if (!manifest_from_json(*manifest_json, impl_->manifest, error)) {
        return false;
    }

    std::ifstream index_file(directory / kIndexName, std::ios::binary);
    if (!index_file) {
        error = "dataset has no " + std::string(kIndexName);
        return false;
    }
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(index_file, line)) {
        ++line_number;
        if (line.empty() || line == "\r") {
            continue;
        }
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        auto entry = json::parse(line, &parse_error);
        if (!entry) {
            error = "malformed index line " + std::to_string(line_number) + ": " + parse_error;
            return false;
        }
        impl_->entries.push_back(std::move(*entry));
    }

    impl_->blob.open(directory / kBlobName, std::ios::binary);
    if (!impl_->blob) {
        error = "dataset has no " + std::string(kBlobName);
        return false;
    }
    return true;
}

auto DatasetReader::manifest() const -> const DatasetManifest& { return impl_->manifest; }

auto DatasetReader::frame_count() const noexcept -> std::size_t { return impl_->entries.size(); }

auto DatasetReader::read(std::size_t index, camera::CameraFrameSet& frame_set,
                         std::string& error) const -> bool {
    error.clear();
    if (index >= impl_->entries.size()) {
        error = "frame index out of range";
        return false;
    }
    const json::Value& entry = impl_->entries[index];
    frame_set = camera::CameraFrameSet{};
    frame_set.capture_timestamp_ns = read_unsigned(entry, "capture_timestamp_ns");

    const auto load_image = [&](const char* key, camera::ImageFrame& image) -> bool {
        const json::Value* node = entry.find(key);
        if (node == nullptr) {
            return true;
        }
        std::vector<std::uint8_t> payload;
        if (!impl_->read_payload(read_unsigned(*node, "offset"), read_unsigned(*node, "bytes"),
                                 payload, error)) {
            return false;
        }
        image_from_json(*node, payload, image);
        return true;
    };

    if (!load_image("rgb", frame_set.rgb) || !load_image("ir_left", frame_set.ir_left) ||
        !load_image("ir_right", frame_set.ir_right)) {
        return false;
    }

    if (const json::Value* depth = entry.find("depth"); depth != nullptr) {
        std::vector<std::uint8_t> payload;
        if (!impl_->read_payload(read_unsigned(*depth, "offset"), read_unsigned(*depth, "bytes"),
                                 payload, error)) {
            return false;
        }
        frame_set.depth.sensor_timestamp_ns = read_unsigned(*depth, "sensor_timestamp_ns");
        frame_set.depth.capture_timestamp_ns = read_unsigned(*depth, "capture_timestamp_ns");
        frame_set.depth.frame_number = read_unsigned(*depth, "frame_number");
        frame_set.depth.width = read_int(*depth, "width");
        frame_set.depth.height = read_int(*depth, "height");
        frame_set.depth.depth_scale_m = static_cast<float>(read_double(*depth, "depth_scale_m"));
        frame_set.depth.data.resize(payload.size() / sizeof(std::uint16_t));
        if (!frame_set.depth.data.empty()) {
            // Z16 is stored in the machine's byte order, as captured; the dataset is a local
            // artefact and never crosses an endianness boundary.
            std::memcpy(frame_set.depth.data.data(), payload.data(), payload.size());
        }
    }

    if (const json::Value* imu = entry.find("imu"); imu != nullptr) {
        camera::ImuBatch batch;
        imu_from_json(*imu, batch);
        frame_set.imu = std::move(batch);
    }
    return true;
}

// -------------------------------------------------------------------------------------------
// Replay camera
// -------------------------------------------------------------------------------------------

class ReplayCamera::Impl {
  public:
    Impl(std::filesystem::path path, Options replay_options)
        : directory(std::move(path)), options(replay_options) {}

    std::filesystem::path directory;
    Options options;
    DatasetReader reader;
    bool opened{false};
    bool running{false};
    std::size_t next_index{0};
    std::uint64_t replayed{0};
    std::uint64_t previous_capture_timestamp_ns{0};
    std::string last_error;
};

ReplayCamera::ReplayCamera(std::filesystem::path directory, Options options)
    : impl_(std::make_unique<Impl>(std::move(directory), options)) {}

ReplayCamera::~ReplayCamera() = default;

auto ReplayCamera::initialize() -> bool {
    if (impl_->opened) {
        return true;
    }
    if (!impl_->reader.open(impl_->directory, impl_->last_error)) {
        return false;
    }
    impl_->opened = true;
    return true;
}

auto ReplayCamera::start() -> bool {
    if (!impl_->opened && !initialize()) {
        return false;
    }
    impl_->running = true;
    impl_->next_index = 0;
    impl_->previous_capture_timestamp_ns = 0;
    return true;
}

void ReplayCamera::stop() noexcept { impl_->running = false; }

auto ReplayCamera::capture(camera::CameraFrameSet& frame_set) -> bool {
    if (!impl_->running) {
        impl_->last_error = "replay camera is not started";
        return false;
    }
    if (impl_->next_index >= impl_->reader.frame_count()) {
        if (!impl_->options.loop || impl_->reader.frame_count() == 0) {
            impl_->last_error = "dataset exhausted";
            return false;
        }
        impl_->next_index = 0;
        impl_->previous_capture_timestamp_ns = 0;
    }

    if (!impl_->reader.read(impl_->next_index, frame_set, impl_->last_error)) {
        return false;
    }
    ++impl_->next_index;
    ++impl_->replayed;

    if (impl_->options.real_time && impl_->previous_capture_timestamp_ns != 0 &&
        frame_set.capture_timestamp_ns > impl_->previous_capture_timestamp_ns) {
        const std::uint64_t delta_ns =
            frame_set.capture_timestamp_ns - impl_->previous_capture_timestamp_ns;
        // Capped so a gap in a recording cannot stall a replay run for minutes.
        constexpr std::uint64_t kMaxSleepNs = 1'000'000'000;
        std::this_thread::sleep_for(std::chrono::nanoseconds(std::min(delta_ns, kMaxSleepNs)));
    }
    impl_->previous_capture_timestamp_ns = frame_set.capture_timestamp_ns;
    return true;
}

auto ReplayCamera::manifest() const -> const DatasetManifest& { return impl_->reader.manifest(); }

auto ReplayCamera::frames_replayed() const noexcept -> std::uint64_t { return impl_->replayed; }

auto ReplayCamera::last_error() const -> std::string { return impl_->last_error; }

} // namespace perception::dataset
