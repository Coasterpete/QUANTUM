#include <quantum/coaster/TrackStyle.hpp>
#include <quantum/renderer/StaticMeshAssets.hpp>

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    using namespace quantum::renderer;

    void require(const bool condition, const char* const message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    void requireNear(
        const float actual,
        const float expected,
        const float tolerance,
        const char* const message)
    {
        require(std::abs(actual - expected) <= tolerance, message);
    }

    [[nodiscard]] std::vector<std::uint8_t> readBytes(
        const std::filesystem::path& path)
    {
        std::ifstream input(path, std::ios::binary | std::ios::ate);
        require(static_cast<bool>(input), "fixture can be opened");
        const std::streamsize size = input.tellg();
        std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
        input.seekg(0);
        input.read(reinterpret_cast<char*>(bytes.data()), size);
        require(static_cast<bool>(input), "fixture can be read");
        return bytes;
    }

    [[nodiscard]] std::uint32_t u32(
        const std::vector<std::uint8_t>& bytes,
        const std::size_t offset)
    {
        std::uint32_t value = 0;
        std::memcpy(&value, bytes.data() + offset, sizeof(value));
        return value;
    }

    void appendU32(std::vector<std::uint8_t>& bytes, const std::uint32_t value)
    {
        const auto* const begin = reinterpret_cast<const std::uint8_t*>(&value);
        bytes.insert(bytes.end(), begin, begin + sizeof(value));
    }

    void writeBytes(
        const std::filesystem::path& path,
        const std::vector<std::uint8_t>& bytes)
    {
        std::ofstream output(path, std::ios::binary | std::ios::trunc);
        output.write(reinterpret_cast<const char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
        require(static_cast<bool>(output), "temporary GLB can be written");
    }

    template<typename Edit>
    void writeEditedGlb(
        const std::filesystem::path& source,
        const std::filesystem::path& destination,
        Edit&& edit)
    {
        constexpr std::uint32_t jsonType = 0x4E4F534Au;
        const std::vector<std::uint8_t> sourceBytes = readBytes(source);
        const std::uint32_t sourceJsonLength = u32(sourceBytes, 12);
        const std::uint32_t sourceJsonType = u32(sourceBytes, 16);
        require(sourceJsonType == jsonType, "fixture JSON chunk is first");
        std::string jsonText(
            reinterpret_cast<const char*>(sourceBytes.data() + 20),
            sourceJsonLength);
        nlohmann::json document = nlohmann::json::parse(jsonText);
        edit(document);
        jsonText = document.dump();
        while (jsonText.size() % 4 != 0)
            jsonText.push_back(' ');

        const std::size_t sourceBinaryHeader = 20 + sourceJsonLength;
        const std::uint32_t binaryLength = u32(sourceBytes, sourceBinaryHeader);
        const std::uint32_t binaryType = u32(sourceBytes, sourceBinaryHeader + 4);
        const std::size_t newLength = 20 + jsonText.size() + 8 + binaryLength;
        require(newLength <= std::numeric_limits<std::uint32_t>::max(),
            "edited fixture remains a valid GLB size");

        std::vector<std::uint8_t> edited;
        edited.reserve(newLength);
        appendU32(edited, 0x46546C67u);
        appendU32(edited, 2u);
        appendU32(edited, static_cast<std::uint32_t>(newLength));
        appendU32(edited, static_cast<std::uint32_t>(jsonText.size()));
        appendU32(edited, jsonType);
        edited.insert(edited.end(), jsonText.begin(), jsonText.end());
        appendU32(edited, binaryLength);
        appendU32(edited, binaryType);
        edited.insert(edited.end(),
            sourceBytes.begin() + sourceBinaryHeader + 8,
            sourceBytes.begin() + sourceBinaryHeader + 8 + binaryLength);
        writeBytes(destination, edited);
    }

    template<typename Operation>
    [[nodiscard]] std::string errorFrom(Operation&& operation)
    {
        try
        {
            operation();
        }
        catch (const std::exception& exception)
        {
            return exception.what();
        }
        throw std::runtime_error("operation unexpectedly succeeded");
    }

    void coordinateConversionIsExplicit()
    {
        const glm::vec3 converted = gltfVectorToQuantum({1.0F, 2.0F, 3.0F});
        require(converted == glm::vec3(1.0F, -3.0F, 2.0F),
            "glTF +Y-up converts to QUANTUM +Z-up");
        requireNear(glm::length(converted), glm::length(glm::vec3(1.0F, 2.0F, 3.0F)),
            1.0e-6F, "axis conversion preserves units and vector length");
    }

    void identifiersNormalizeAndRejectEscapes()
    {
        require(normalizeStaticMeshAssetIdentifier(
                "assets://track\\.\\test-crosstie-placeholder.glb")
                == "assets://track/test-crosstie-placeholder.glb",
            "asset separators and dot components normalize");
        const std::string message = errorFrom([]
        {
            static_cast<void>(normalizeStaticMeshAssetIdentifier(
                "assets://../outside.glb"));
        });
        require(message.find("cannot escape") != std::string::npos,
            "asset identity rejects package-root traversal");
    }

    void validFixtureLoadsDeterministically(
        const std::filesystem::path& runtimeRoot)
    {
        constexpr std::string_view identifier =
            "assets://track/test-crosstie-placeholder.glb";
        const std::filesystem::path path = runtimeRoot / "assets" / "track"
            / "test-crosstie-placeholder.glb";
        const StaticMeshAsset first = loadStaticMeshGlb(
            std::string(identifier), path);
        const StaticMeshAsset second = loadStaticMeshGlb(
            std::string(identifier), path);

        require(first.identifier == identifier, "logical asset identity retained");
        require(first.vertices.size() == 96, "fixture vertex count is deterministic");
        require(first.triangleIndices.size() == 132,
            "fixture triangle-index count is deterministic");
        require(first.triangleIndices == second.triangleIndices,
            "repeated GLB loads produce identical indices");
        require(first.edgeIndices == second.edgeIndices,
            "repeated GLB loads produce identical edges");
        require(first.submeshes.size() == 1,
            "single fixture primitive is retained as one submesh");

        glm::vec3 minimum{std::numeric_limits<float>::max()};
        glm::vec3 maximum{std::numeric_limits<float>::lowest()};
        for (std::size_t index = 0; index < first.vertices.size(); ++index)
        {
            const StaticMeshVertex& vertex = first.vertices[index];
            const StaticMeshVertex& repeated = second.vertices[index];
            require(vertex.position == repeated.position
                    && vertex.normal == repeated.normal,
                "repeated GLB loads produce identical vertices");
            require(std::isfinite(vertex.position.x)
                    && std::isfinite(vertex.position.y)
                    && std::isfinite(vertex.position.z),
                "fixture positions are finite");
            require(std::isfinite(vertex.normal.x)
                    && std::isfinite(vertex.normal.y)
                    && std::isfinite(vertex.normal.z),
                "fixture normals are finite");
            requireNear(glm::length(vertex.normal), 1.0F, 1.0e-5F,
                "fixture normals are normalized");
            minimum = glm::min(minimum, vertex.position);
            maximum = glm::max(maximum, vertex.position);
        }
        for (const std::uint32_t index : first.triangleIndices)
            require(index < first.vertices.size(), "fixture indices are in bounds");

        requireNear(minimum.x, -0.09F, 1.0e-5F,
            "meter-authored forward half-extent is preserved");
        requireNear(maximum.y, 0.70F, 1.0e-5F,
            "Blender lateral extent maps to QUANTUM local Y");
        requireNear(maximum.z, 0.06F, 1.0e-5F,
            "Blender up extent maps to QUANTUM local Z");
    }

    void cpuAndGpuCachesReuseAssets(const std::filesystem::path& runtimeRoot)
    {
        constexpr std::string_view identifier =
            "assets://track/test-crosstie-placeholder.glb";
        StaticMeshAssetCache cpuCache{runtimeRoot};
        const auto first = cpuCache.load(identifier);
        const auto repeated = cpuCache.load(
            "assets://track/./test-crosstie-placeholder.glb");
        require(first == repeated && cpuCache.size() == 1,
            "normalized repeated CPU lookup reuses one immutable asset");

        quantum::coaster::HardwareInstanceBatch batch;
        batch.asset.path = std::string(identifier);
        batch.instances.emplace_back();
        require(cpuCache.load(batch.asset.path) == first,
            "HardwareInstanceBatch renderer-neutral asset reference resolves");

        int uploadCount = 0;
        StaticMeshGpuHandleCache gpuCache;
        const auto upload = [&uploadCount](const StaticMeshAsset&)
        {
            ++uploadCount;
            return StaticMeshGpuHandle{7};
        };
        const StaticMeshGpuHandle firstHandle =
            gpuCache.getOrUpload(*first, upload);
        const StaticMeshGpuHandle repeatedHandle =
            gpuCache.getOrUpload(*repeated, upload);
        require(firstHandle == repeatedHandle && firstHandle.value == 7
                && uploadCount == 1 && gpuCache.size() == 1,
            "repeated GPU requests invoke one upload and reuse its handle");

        const auto builtin = cpuCache.load(diagnosticHardwareAssetId);
        require(builtin->vertices.size() == 8
                && builtin->triangleIndices.size() == 36,
            "builtin diagnostic hardware asset remains available");

        static_cast<void>(gpuCache.getOrUpload(*builtin, upload));
        require(cpuCache.size() == 2 && gpuCache.size() == 2,
            "two distinct assets occupy two cache entries");
        require(cpuCache.invalidate(identifier) && cpuCache.size() == 1,
            "CPU reload invalidates exactly the referenced cache entry");
        const auto removedHandle = gpuCache.invalidate(identifier);
        require(removedHandle.has_value() && gpuCache.size() == 1,
            "GPU reload invalidates exactly the referenced cache entry");
        const int uploadsBeforeBuiltinLookup = uploadCount;
        require(gpuCache.getOrUpload(*builtin, upload).value == 7
                && uploadCount == uploadsBeforeBuiltinLookup,
            "invalidating one GPU entry preserves every other cached asset");
        require(cpuCache.load(diagnosticHardwareAssetId) == builtin,
            "invalidating one CPU entry preserves every other cached asset");
    }

    void invalidAssetsReportTheirLogicalIdentity(
        const std::filesystem::path& runtimeRoot)
    {
        constexpr std::string_view missing = "assets://track/missing.glb";
        StaticMeshAssetCache cache{runtimeRoot};
        const std::string missingMessage = errorFrom([&]
        {
            static_cast<void>(cache.load(missing));
        });
        require(missingMessage.find(missing) != std::string::npos
                && missingMessage.find("file not found") != std::string::npos,
            "missing file error includes logical identity and cause");
        require(classifyStaticMeshLoadFailure(missingMessage)
                == HardwareAssetLoadState::MissingAsset,
            "missing asset failure maps to concise UI status");

        quantum::coaster::HardwareInstanceBatch missingBatch;
        missingBatch.asset.path = std::string(missing);
        try
        {
            static_cast<void>(cache.load(missingBatch.asset.path));
        }
        catch (const std::exception&)
        {
        }
        require(missingBatch.asset.path == missing,
            "load failure and fallback handling preserve the selected logical ID");

        const auto unique = std::chrono::steady_clock::now()
            .time_since_epoch().count();
        const std::filesystem::path temporary =
            std::filesystem::temp_directory_path()
            / ("quantum-static-mesh-tests-" + std::to_string(unique));
        std::filesystem::create_directories(temporary);
        const std::filesystem::path malformed = temporary / "malformed.glb";
        writeBytes(malformed, {0, 1, 2, 3});
        const std::string malformedMessage = errorFrom([&]
        {
            static_cast<void>(loadStaticMeshGlb(
                "assets://tests/malformed.glb", malformed));
        });
        require(malformedMessage.find("assets://tests/malformed.glb")
                    != std::string::npos
                && malformedMessage.find("invalid GLB") != std::string::npos,
            "malformed GLB error includes logical identity");
        require(classifyStaticMeshLoadFailure(malformedMessage)
                == HardwareAssetLoadState::InvalidGlb,
            "malformed asset failure maps to invalid GLB UI status");

        const std::filesystem::path fixture = runtimeRoot / "assets" / "track"
            / "test-crosstie-placeholder.glb";
        const std::filesystem::path unsupported = temporary / "lines.glb";
        writeEditedGlb(fixture, unsupported, [](nlohmann::json& document)
        {
            document["meshes"][0]["primitives"][0]["mode"] = 1;
        });
        const std::string unsupportedMessage = errorFrom([&]
        {
            static_cast<void>(loadStaticMeshGlb(
                "assets://tests/lines.glb", unsupported));
        });
        require(unsupportedMessage.find("primitive mode is unsupported")
                != std::string::npos,
            "non-triangle primitive is rejected clearly");
        require(classifyStaticMeshLoadFailure(unsupportedMessage)
                == HardwareAssetLoadState::UnsupportedGlb,
            "unsupported asset subset maps to concise UI status");

        const std::filesystem::path noNormals = temporary / "no-normals.glb";
        writeEditedGlb(fixture, noNormals, [](nlohmann::json& document)
        {
            document["meshes"][0]["primitives"][0]["attributes"].erase("NORMAL");
        });
        const std::string normalMessage = errorFrom([&]
        {
            static_cast<void>(loadStaticMeshGlb(
                "assets://tests/no-normals.glb", noNormals));
        });
        require(normalMessage.find("no NORMAL attribute") != std::string::npos,
            "missing normals follow the documented reject policy");

        std::error_code cleanupError;
        std::filesystem::remove_all(temporary, cleanupError);
    }
}

int main(int argc, char* argv[])
{
    try
    {
        require(argc == 2, "pass the QUANTUM runtime/source root");
        const std::filesystem::path runtimeRoot = argv[1];
        coordinateConversionIsExplicit();
        identifiersNormalizeAndRejectEscapes();
        validFixtureLoadsDeterministically(runtimeRoot);
        cpuAndGpuCachesReuseAssets(runtimeRoot);
        invalidAssetsReportTheirLogicalIdentity(runtimeRoot);
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Static mesh asset test failure: "
            << exception.what() << '\n';
        return 1;
    }

    std::cout << "Static mesh asset tests passed.\n";
    return 0;
}
