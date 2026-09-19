#pragma once

#include "perception/camera/camera.hpp"
#include "perception/camera/camera_info.hpp"
#include "perception/camera/camera_types.hpp"
#include "perception/geometry/transform.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// On-disk capture datasets, so a field failure can be replayed offline and become a regression
// test. Everything downstream of capture consumes `ICamera`, so a replayed dataset is
// indistinguishable from a live D435i to the depth, VO/VIO and localization branches.
//
// Layout (format version 1):
//
//   <directory>/manifest.json   device, intrinsics, mount extrinsics, stream configuration
//   <directory>/index.jsonl     one JSON object per frame set, in capture order
//   <directory>/blob.bin        concatenated raw payloads referenced by offset and length
//
// Image and depth payloads are stored byte-for-byte as captured. Metric Z16 depth is never
// re-encoded: it is measurement data, not video.
namespace perception::dataset {

inline constexpr int kFormatVersion = 1;

struct DatasetManifest {
    int format_version{kFormatVersion};
    std::string created_utc;
    camera::CameraDeviceInfo device;
    std::optional<camera::CameraIntrinsics> rgb_intrinsics;
    std::optional<camera::CameraIntrinsics> depth_intrinsics;
    geometry::MountExtrinsics mount;
    std::string notes;
};

// Picks the intrinsics a recording should store for one stream.
//
// Matching is case-insensitive because librealsense reports stream names capitalized ("Color",
// "Depth", "Infrared") while configuration and call sites spell them in lower case; comparing them
// literally silently matches nothing and produces a dataset that cannot be deprojected.
//
// A profile at the configured resolution wins. `exact_resolution` reports whether such a profile
// was found, so a caller can refuse or warn rather than deproject with intrinsics belonging to a
// different resolution, where fx and cx are simply wrong.
[[nodiscard]] auto select_stream_intrinsics(const camera::CameraDeviceInfo& device,
                                            std::string_view stream, int width, int height,
                                            bool* exact_resolution = nullptr)
    -> std::optional<camera::CameraIntrinsics>;

class DatasetWriter final {
  public:
    DatasetWriter();
    ~DatasetWriter();

    DatasetWriter(const DatasetWriter&) = delete;
    auto operator=(const DatasetWriter&) -> DatasetWriter& = delete;
    DatasetWriter(DatasetWriter&&) = delete;
    auto operator=(DatasetWriter&&) -> DatasetWriter& = delete;

    // Creates the directory and opens the blob and index files. Refuses to overwrite an existing
    // dataset: recordings are field evidence, not scratch files.
    [[nodiscard]] auto open(const std::filesystem::path& directory, const DatasetManifest& manifest,
                            std::string& error) -> bool;
    [[nodiscard]] auto append(const camera::CameraFrameSet& frame_set, std::string& error) -> bool;
    [[nodiscard]] auto close(std::string& error) -> bool;

    [[nodiscard]] auto frames_written() const noexcept -> std::uint64_t;
    [[nodiscard]] auto bytes_written() const noexcept -> std::uint64_t;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

class DatasetReader final {
  public:
    DatasetReader();
    ~DatasetReader();

    DatasetReader(const DatasetReader&) = delete;
    auto operator=(const DatasetReader&) -> DatasetReader& = delete;
    DatasetReader(DatasetReader&&) = delete;
    auto operator=(DatasetReader&&) -> DatasetReader& = delete;

    [[nodiscard]] auto open(const std::filesystem::path& directory, std::string& error) -> bool;
    [[nodiscard]] auto manifest() const -> const DatasetManifest&;
    [[nodiscard]] auto frame_count() const noexcept -> std::size_t;
    [[nodiscard]] auto read(std::size_t index, camera::CameraFrameSet& frame_set,
                            std::string& error) const -> bool;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

// Replays a dataset through the camera interface.
class ReplayCamera final : public camera::ICamera {
  public:
    struct Options {
        // Restart from the first frame after the last one instead of reporting exhaustion.
        bool loop{false};
        // Pace `capture` using the recorded capture timestamps instead of returning immediately.
        bool real_time{false};
    };

    ReplayCamera(std::filesystem::path directory, Options options);
    ~ReplayCamera() override;

    ReplayCamera(const ReplayCamera&) = delete;
    auto operator=(const ReplayCamera&) -> ReplayCamera& = delete;
    ReplayCamera(ReplayCamera&&) = delete;
    auto operator=(ReplayCamera&&) -> ReplayCamera& = delete;

    [[nodiscard]] auto initialize() -> bool override;
    [[nodiscard]] auto start() -> bool override;
    void stop() noexcept override;
    // Returns false at the end of a non-looping dataset, which is how a replay run terminates.
    [[nodiscard]] auto capture(camera::CameraFrameSet& frame_set) -> bool override;

    [[nodiscard]] auto manifest() const -> const DatasetManifest&;
    [[nodiscard]] auto frames_replayed() const noexcept -> std::uint64_t;
    [[nodiscard]] auto last_error() const -> std::string;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace perception::dataset
