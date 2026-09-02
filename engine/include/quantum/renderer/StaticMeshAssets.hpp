#pragma once

#include <glm/vec3.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace quantum::renderer
{
    inline constexpr std::string_view diagnosticHardwareAssetId =
        "builtin://diagnostic/track-hardware-placeholder";

    struct StaticMeshVertex
    {
        glm::vec3 position{0.0F};
        glm::vec3 normal{0.0F, 0.0F, 1.0F};
    };

    struct StaticMeshSubmesh
    {
        std::uint32_t firstIndex = 0;
        std::uint32_t indexCount = 0;
    };

    // Immutable after publication through StaticMeshAssetCache. Materials are
    // deliberately absent: the existing track-hardware material remains the
    // rendering authority for this milestone.
    struct StaticMeshAsset
    {
        std::string identifier;
        std::vector<StaticMeshVertex> vertices;
        std::vector<std::uint32_t> triangleIndices;
        std::vector<std::uint32_t> edgeIndices;
        std::vector<StaticMeshSubmesh> submeshes;
    };

    enum class HardwareAssetLoadState
    {
        Loaded,
        MissingAsset,
        InvalidGlb,
        UnsupportedGlb,
        LoadFailed
    };

    struct HardwareAssetLoadStatus
    {
        std::string requestedIdentifier;
        HardwareAssetLoadState state = HardwareAssetLoadState::Loaded;
        bool usingDiagnosticFallback = false;
        std::string detail;
    };

    [[nodiscard]] HardwareAssetLoadState classifyStaticMeshLoadFailure(
        std::string_view message) noexcept;
    [[nodiscard]] const char* hardwareAssetLoadStateName(
        HardwareAssetLoadState state) noexcept;

    // Converts glTF's right-handed +Y-up coordinates back to QUANTUM's
    // track-local +X-forward, +Y-lateral, +Z-up convention used for Blender
    // authoring. This is a proper rotation, so triangle winding is preserved.
    [[nodiscard]] glm::vec3 gltfVectorToQuantum(
        const glm::vec3& vector) noexcept;

    [[nodiscard]] std::string normalizeStaticMeshAssetIdentifier(
        std::string_view identifier);

    [[nodiscard]] StaticMeshAsset loadStaticMeshGlb(
        std::string identifier,
        const std::filesystem::path& path);

    class StaticMeshAssetCache
    {
    public:
        StaticMeshAssetCache() = default;
        explicit StaticMeshAssetCache(std::filesystem::path runtimeRoot);

        void setRuntimeRoot(std::filesystem::path runtimeRoot);

        [[nodiscard]] std::shared_ptr<const StaticMeshAsset> load(
            std::string_view identifier);
        // Removes exactly one normalized cache entry. It does not affect any
        // other asset and returns whether the entry was present.
        bool invalidate(std::string_view identifier);
        [[nodiscard]] std::size_t size() const noexcept;
        [[nodiscard]] const std::filesystem::path& runtimeRoot() const noexcept;

    private:
        std::filesystem::path runtimeRoot_;
        std::unordered_map<
            std::string,
            std::shared_ptr<const StaticMeshAsset>> assets_;
    };

    struct StaticMeshGpuHandle
    {
        std::uint32_t value = std::numeric_limits<std::uint32_t>::max();

        [[nodiscard]] explicit operator bool() const noexcept
        {
            return value != std::numeric_limits<std::uint32_t>::max();
        }

        friend bool operator==(
            StaticMeshGpuHandle,
            StaticMeshGpuHandle) = default;
    };

    // VulkanContext owns the actual GPU allocations. This cache only maps a
    // normalized CPU asset identity to its renderer-owned handle and invokes
    // the upload operation exactly once after a successful upload.
    class StaticMeshGpuHandleCache
    {
    public:
        using Upload = std::function<StaticMeshGpuHandle(
            const StaticMeshAsset&)>;

        [[nodiscard]] StaticMeshGpuHandle getOrUpload(
            const StaticMeshAsset& asset,
            const Upload& upload);
        [[nodiscard]] std::size_t size() const noexcept;
        [[nodiscard]] std::optional<StaticMeshGpuHandle> invalidate(
            std::string_view identifier) noexcept;
        void clear() noexcept;

    private:
        std::unordered_map<std::string, StaticMeshGpuHandle> handles_;
    };
}
