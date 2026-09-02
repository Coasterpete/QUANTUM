#include <quantum/renderer/StaticMeshAssets.hpp>

#include <glm/geometric.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using Json = nlohmann::json;
    using quantum::renderer::StaticMeshAsset;
    using quantum::renderer::StaticMeshSubmesh;
    using quantum::renderer::StaticMeshVertex;

    constexpr std::uint32_t glbMagic = 0x46546C67u;
    constexpr std::uint32_t glbVersion = 2u;
    constexpr std::uint32_t jsonChunkType = 0x4E4F534Au;
    constexpr std::uint32_t binaryChunkType = 0x004E4942u;
    constexpr std::uint32_t trianglePrimitiveMode = 4u;
    constexpr std::uint32_t componentUnsignedByte = 5121u;
    constexpr std::uint32_t componentUnsignedShort = 5123u;
    constexpr std::uint32_t componentUnsignedInt = 5125u;
    constexpr std::uint32_t componentFloat = 5126u;

    [[noreturn]] void assetError(
        const std::string_view identifier,
        const std::string_view message)
    {
        throw std::runtime_error(
            "Static mesh asset '" + std::string(identifier) + "': "
            + std::string(message));
    }

    [[nodiscard]] std::size_t checkedAdd(
        const std::size_t left,
        const std::size_t right,
        const std::string_view identifier,
        const std::string_view context)
    {
        if (right > std::numeric_limits<std::size_t>::max() - left)
        {
            assetError(identifier, std::string(context) + " range overflows.");
        }
        return left + right;
    }

    [[nodiscard]] std::size_t checkedMultiply(
        const std::size_t left,
        const std::size_t right,
        const std::string_view identifier,
        const std::string_view context)
    {
        if (left != 0
            && right > std::numeric_limits<std::size_t>::max() / left)
        {
            assetError(identifier, std::string(context) + " size overflows.");
        }
        return left * right;
    }

    [[nodiscard]] std::uint32_t readU32(
        const std::span<const std::byte> bytes,
        const std::size_t offset,
        const std::string_view identifier,
        const std::string_view context)
    {
        if (offset > bytes.size() || bytes.size() - offset < sizeof(std::uint32_t))
        {
            assetError(identifier, std::string(context) + " is truncated.");
        }
        std::uint32_t result = 0;
        std::memcpy(&result, bytes.data() + offset, sizeof(result));
        if constexpr (std::endian::native == std::endian::big)
        {
            result = std::byteswap(result);
        }
        return result;
    }

    [[nodiscard]] std::vector<std::byte> readFile(
        const std::filesystem::path& path,
        const std::string_view identifier)
    {
        std::ifstream input(path, std::ios::binary | std::ios::ate);
        if (!input)
        {
            assetError(identifier, "file not found at " + path.string() + ".");
        }
        const std::streampos end = input.tellg();
        if (end <= 0)
        {
            assetError(identifier, "file is empty or unreadable.");
        }
        const auto fileSize = static_cast<std::uintmax_t>(end);
        if (fileSize > std::numeric_limits<std::size_t>::max())
        {
            assetError(identifier, "file is too large for this platform.");
        }
        std::vector<std::byte> bytes(static_cast<std::size_t>(fileSize));
        input.seekg(0);
        input.read(
            reinterpret_cast<char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
        if (!input)
        {
            assetError(identifier, "file could not be read completely.");
        }
        return bytes;
    }

    struct GlbContents
    {
        Json document;
        std::span<const std::byte> binary;
    };

    [[nodiscard]] GlbContents parseGlb(
        const std::span<const std::byte> bytes,
        const std::string_view identifier)
    {
        if (bytes.size() < 20)
        {
            assetError(identifier, "invalid GLB header.");
        }
        if (readU32(bytes, 0, identifier, "GLB magic") != glbMagic)
        {
            assetError(identifier, "invalid GLB magic; only binary .glb files are supported.");
        }
        if (readU32(bytes, 4, identifier, "GLB version") != glbVersion)
        {
            assetError(identifier, "unsupported GLB version; expected glTF 2.0.");
        }
        if (readU32(bytes, 8, identifier, "GLB length") != bytes.size())
        {
            assetError(identifier, "GLB declared length does not match the file size.");
        }

        std::span<const std::byte> jsonBytes;
        std::span<const std::byte> binaryBytes;
        std::size_t offset = 12;
        std::size_t chunkIndex = 0;
        while (offset < bytes.size())
        {
            if (bytes.size() - offset < 8)
            {
                assetError(identifier, "GLB chunk header is truncated.");
            }
            const std::uint32_t length = readU32(
                bytes, offset, identifier, "GLB chunk length");
            const std::uint32_t type = readU32(
                bytes, offset + 4, identifier, "GLB chunk type");
            offset += 8;
            const std::size_t chunkEnd = checkedAdd(
                offset, length, identifier, "GLB chunk");
            if (chunkEnd > bytes.size())
            {
                assetError(identifier, "GLB chunk extends beyond the file.");
            }
            const std::span<const std::byte> contents = bytes.subspan(offset, length);
            if (chunkIndex == 0 && type != jsonChunkType)
            {
                assetError(identifier, "the first GLB chunk must contain JSON.");
            }
            if (type == jsonChunkType)
            {
                if (!jsonBytes.empty())
                {
                    assetError(identifier, "GLB contains more than one JSON chunk.");
                }
                jsonBytes = contents;
            }
            else if (type == binaryChunkType)
            {
                if (!binaryBytes.empty())
                {
                    assetError(identifier, "GLB contains more than one binary chunk.");
                }
                binaryBytes = contents;
            }
            else
            {
                assetError(identifier, "GLB contains an unsupported chunk type.");
            }
            offset = chunkEnd;
            ++chunkIndex;
        }
        if (jsonBytes.empty() || binaryBytes.empty())
        {
            assetError(identifier, "GLB requires one JSON chunk and one binary chunk.");
        }

        Json document;
        try
        {
            const auto* begin = reinterpret_cast<const char*>(jsonBytes.data());
            document = Json::parse(begin, begin + jsonBytes.size());
        }
        catch (const Json::exception& exception)
        {
            assetError(identifier, "malformed glTF JSON: " + std::string(exception.what()));
        }
        return {std::move(document), binaryBytes};
    }

    void requireArrayAbsentOrEmpty(
        const Json& document,
        const char* const member,
        const std::string_view identifier)
    {
        const auto found = document.find(member);
        if (found != document.end() && (!found->is_array() || !found->empty()))
        {
            assetError(identifier, std::string(member) + " are not supported.");
        }
    }

    void validateDocumentSubset(
        const Json& document,
        const std::string_view identifier)
    {
        if (!document.is_object())
        {
            assetError(identifier, "glTF root must be an object.");
        }
        const auto asset = document.find("asset");
        if (asset == document.end() || !asset->is_object()
            || asset->value("version", std::string{}) != "2.0")
        {
            assetError(identifier, "glTF asset.version must be 2.0.");
        }
        requireArrayAbsentOrEmpty(document, "animations", identifier);
        requireArrayAbsentOrEmpty(document, "skins", identifier);
        requireArrayAbsentOrEmpty(document, "cameras", identifier);

        const auto meshes = document.find("meshes");
        if (meshes == document.end() || !meshes->is_array() || meshes->empty())
        {
            assetError(identifier, "no mesh is present.");
        }
        if (meshes->size() != 1)
        {
            assetError(identifier, "exactly one mesh is supported in this milestone.");
        }

        const auto nodes = document.find("nodes");
        if (nodes != document.end())
        {
            if (!nodes->is_array() || nodes->size() != 1)
            {
                assetError(identifier,
                    "exactly one untransformed mesh node is supported.");
            }
            std::size_t meshNodeCount = 0;
            for (const Json& node : *nodes)
            {
                if (!node.is_object())
                {
                    assetError(identifier, "a node is malformed.");
                }
                for (const char* const transform : {
                    "matrix", "translation", "rotation", "scale", "children",
                    "camera", "skin", "extensions"})
                {
                    if (node.contains(transform))
                    {
                        assetError(identifier,
                            "node transforms/hierarchy are unsupported; apply object transforms before Blender export.");
                    }
                }
                if (node.contains("mesh"))
                {
                    if (!node["mesh"].is_number_unsigned()
                        || node["mesh"].get<std::size_t>() != 0)
                    {
                        assetError(identifier, "node references an unsupported mesh.");
                    }
                    ++meshNodeCount;
                }
            }
            if (meshNodeCount != 1)
            {
                assetError(identifier,
                    "the single glTF node must reference the imported mesh.");
            }
        }
    }

    struct AccessorView
    {
        const std::byte* data = nullptr;
        std::size_t count = 0;
        std::size_t stride = 0;
        std::uint32_t componentType = 0;
    };

    [[nodiscard]] std::size_t componentSize(
        const std::uint32_t componentType,
        const std::string_view identifier)
    {
        switch (componentType)
        {
        case componentUnsignedByte:
            return 1;
        case componentUnsignedShort:
            return 2;
        case componentUnsignedInt:
        case componentFloat:
            return 4;
        default:
            assetError(identifier, "accessor uses an unsupported component type.");
        }
    }

    [[nodiscard]] AccessorView accessorView(
        const Json& document,
        const std::span<const std::byte> binary,
        const std::size_t accessorIndex,
        const std::string_view expectedType,
        const std::uint32_t expectedComponent,
        const std::string_view identifier)
    {
        const Json& accessors = document.at("accessors");
        const Json& bufferViews = document.at("bufferViews");
        if (!accessors.is_array() || accessorIndex >= accessors.size())
        {
            assetError(identifier, "primitive references an invalid accessor.");
        }
        const Json& accessor = accessors[accessorIndex];
        if (!accessor.is_object() || accessor.contains("sparse"))
        {
            assetError(identifier, "sparse or malformed accessors are unsupported.");
        }
        if (accessor.value("type", std::string{}) != expectedType
            || accessor.value("componentType", 0u) != expectedComponent
            || accessor.value("normalized", false))
        {
            assetError(identifier, "accessor type does not match the required mesh attribute format.");
        }
        if (!accessor.contains("bufferView")
            || !accessor["bufferView"].is_number_unsigned()
            || !accessor.contains("count")
            || !accessor["count"].is_number_unsigned())
        {
            assetError(identifier, "accessor is missing bufferView or count.");
        }
        const std::size_t viewIndex = accessor["bufferView"].get<std::size_t>();
        if (!bufferViews.is_array() || viewIndex >= bufferViews.size())
        {
            assetError(identifier, "accessor references an invalid bufferView.");
        }
        const Json& view = bufferViews[viewIndex];
        if (!view.is_object() || view.value("buffer", std::size_t{1}) != 0
            || !view.contains("byteLength")
            || !view["byteLength"].is_number_unsigned())
        {
            assetError(identifier, "bufferView must reference the embedded GLB buffer.");
        }

        const std::size_t components = expectedType == "VEC3" ? 3 : 1;
        const std::size_t elementSize = checkedMultiply(
            componentSize(expectedComponent, identifier), components,
            identifier, "accessor element");
        const std::size_t stride = view.value("byteStride", elementSize);
        if (stride < elementSize || stride % componentSize(expectedComponent, identifier) != 0)
        {
            assetError(identifier, "bufferView byteStride is invalid.");
        }
        const std::size_t count = accessor["count"].get<std::size_t>();
        if (count == 0)
        {
            assetError(identifier, "accessor count must be nonzero.");
        }
        const std::size_t viewOffset = view.value("byteOffset", std::size_t{0});
        const std::size_t accessorOffset = accessor.value("byteOffset", std::size_t{0});
        const std::size_t viewLength = view["byteLength"].get<std::size_t>();
        const std::size_t occupied = checkedAdd(
            checkedMultiply(count - 1, stride, identifier, "accessor"),
            elementSize, identifier, "accessor");
        if (accessorOffset > viewLength || occupied > viewLength - accessorOffset)
        {
            assetError(identifier, "accessor exceeds its bufferView.");
        }
        const std::size_t begin = checkedAdd(
            viewOffset, accessorOffset, identifier, "accessor");
        if (begin > binary.size() || occupied > binary.size() - begin)
        {
            assetError(identifier, "accessor exceeds the embedded GLB buffer.");
        }
        return {binary.data() + begin, count, stride, expectedComponent};
    }

    [[nodiscard]] AccessorView indexAccessorView(
        const Json& document,
        const std::span<const std::byte> binary,
        const std::size_t accessorIndex,
        const std::string_view identifier)
    {
        const Json& accessors = document.at("accessors");
        if (!accessors.is_array() || accessorIndex >= accessors.size()
            || !accessors[accessorIndex].is_object())
        {
            assetError(identifier, "primitive references an invalid index accessor.");
        }
        const std::uint32_t component =
            accessors[accessorIndex].value("componentType", 0u);
        if (component != componentUnsignedByte
            && component != componentUnsignedShort
            && component != componentUnsignedInt)
        {
            assetError(identifier, "indices must use an unsigned integer component type.");
        }
        return accessorView(document, binary, accessorIndex, "SCALAR",
            component, identifier);
    }

    [[nodiscard]] float readFloat(const std::byte* source) noexcept
    {
        std::uint32_t bits = 0;
        std::memcpy(&bits, source, sizeof(bits));
        if constexpr (std::endian::native == std::endian::big)
        {
            bits = std::byteswap(bits);
        }
        return std::bit_cast<float>(bits);
    }

    [[nodiscard]] glm::vec3 readVec3(
        const AccessorView& view,
        const std::size_t index) noexcept
    {
        const std::byte* const source = view.data + index * view.stride;
        return {readFloat(source), readFloat(source + 4), readFloat(source + 8)};
    }

    [[nodiscard]] std::uint32_t readIndex(
        const AccessorView& view,
        const std::size_t index) noexcept
    {
        const std::byte* const source = view.data + index * view.stride;
        switch (view.componentType)
        {
        case componentUnsignedByte:
            return std::to_integer<std::uint8_t>(*source);
        case componentUnsignedShort:
        {
            std::uint16_t value = 0;
            std::memcpy(&value, source, sizeof(value));
            if constexpr (std::endian::native == std::endian::big)
                value = std::byteswap(value);
            return value;
        }
        case componentUnsignedInt:
        {
            std::uint32_t value = 0;
            std::memcpy(&value, source, sizeof(value));
            if constexpr (std::endian::native == std::endian::big)
                value = std::byteswap(value);
            return value;
        }
        default:
            return 0;
        }
    }

    [[nodiscard]] bool finite(const glm::vec3& vector) noexcept
    {
        return std::isfinite(vector.x) && std::isfinite(vector.y)
            && std::isfinite(vector.z);
    }

    void createEdges(StaticMeshAsset& asset)
    {
        std::set<std::pair<std::uint32_t, std::uint32_t>> edges;
        const auto add = [&edges](std::uint32_t first, std::uint32_t second)
        {
            if (first > second)
                std::swap(first, second);
            edges.emplace(first, second);
        };
        for (std::size_t index = 0; index < asset.triangleIndices.size(); index += 3)
        {
            const std::uint32_t a = asset.triangleIndices[index];
            const std::uint32_t b = asset.triangleIndices[index + 1];
            const std::uint32_t c = asset.triangleIndices[index + 2];
            add(a, b);
            add(b, c);
            add(c, a);
        }
        asset.edgeIndices.reserve(edges.size() * 2);
        for (const auto [first, second] : edges)
        {
            asset.edgeIndices.push_back(first);
            asset.edgeIndices.push_back(second);
        }
    }

    [[nodiscard]] StaticMeshAsset diagnosticHardwareMesh()
    {
        using quantum::renderer::diagnosticHardwareAssetId;
        constexpr float n = 0.57735026919F;
        StaticMeshAsset asset;
        asset.identifier = diagnosticHardwareAssetId;
        asset.vertices = {
            {{-0.5F, -0.5F, -0.5F}, {-n, -n, -n}},
            {{ 0.5F, -0.5F, -0.5F}, { n, -n, -n}},
            {{ 0.5F,  0.5F, -0.5F}, { n,  n, -n}},
            {{-0.5F,  0.5F, -0.5F}, {-n,  n, -n}},
            {{-0.5F, -0.5F,  0.5F}, {-n, -n,  n}},
            {{ 0.5F, -0.5F,  0.5F}, { n, -n,  n}},
            {{ 0.5F,  0.5F,  0.5F}, { n,  n,  n}},
            {{-0.5F,  0.5F,  0.5F}, {-n,  n,  n}}
        };
        asset.triangleIndices = {
            0, 2, 1, 0, 3, 2,
            4, 5, 6, 4, 6, 7,
            0, 1, 5, 0, 5, 4,
            1, 2, 6, 1, 6, 5,
            2, 3, 7, 2, 7, 6,
            3, 0, 4, 3, 4, 7
        };
        asset.submeshes.push_back({0,
            static_cast<std::uint32_t>(asset.triangleIndices.size())});
        createEdges(asset);
        return asset;
    }
}

namespace quantum::renderer
{
    HardwareAssetLoadState classifyStaticMeshLoadFailure(
        const std::string_view message) noexcept
    {
        if (message.find("file not found") != std::string_view::npos)
            return HardwareAssetLoadState::MissingAsset;
        if (message.find("unsupported") != std::string_view::npos
            || message.find("only .glb") != std::string_view::npos)
            return HardwareAssetLoadState::UnsupportedGlb;
        if (message.find("invalid GLB") != std::string_view::npos
            || message.find("malformed glTF") != std::string_view::npos)
            return HardwareAssetLoadState::InvalidGlb;
        return HardwareAssetLoadState::LoadFailed;
    }

    const char* hardwareAssetLoadStateName(
        const HardwareAssetLoadState state) noexcept
    {
        switch (state)
        {
        case HardwareAssetLoadState::Loaded: return "Loaded";
        case HardwareAssetLoadState::MissingAsset: return "Missing asset";
        case HardwareAssetLoadState::InvalidGlb: return "Invalid GLB";
        case HardwareAssetLoadState::UnsupportedGlb: return "Unsupported GLB";
        case HardwareAssetLoadState::LoadFailed: return "Load failed";
        }
        return "Load failed";
    }

    glm::vec3 gltfVectorToQuantum(const glm::vec3& vector) noexcept
    {
        return {vector.x, -vector.z, vector.y};
    }

    std::string normalizeStaticMeshAssetIdentifier(
        const std::string_view identifier)
    {
        if (identifier.empty())
        {
            throw std::invalid_argument("Static mesh asset identifier is empty.");
        }
        std::string normalized{identifier};
        std::ranges::replace(normalized, '\\', '/');

        constexpr std::string_view assetsScheme = "assets://";
        constexpr std::string_view builtinScheme = "builtin://";
        std::string_view scheme;
        if (normalized.starts_with(assetsScheme))
            scheme = assetsScheme;
        else if (normalized.starts_with(builtinScheme))
            scheme = builtinScheme;
        else
            throw std::invalid_argument(
                "Static mesh asset identifier must use assets:// or builtin://: "
                + normalized);

        const std::filesystem::path relative = std::filesystem::path(
            normalized.substr(scheme.size())).lexically_normal();
        if (relative.empty() || relative.is_absolute()
            || relative.has_root_name() || relative.has_root_directory())
        {
            throw std::invalid_argument(
                "Static mesh asset identifier has no valid package-relative path: "
                + normalized);
        }
        for (const std::filesystem::path& part : relative)
        {
            if (part == "..")
            {
                throw std::invalid_argument(
                    "Static mesh asset identifier cannot escape the package root: "
                    + normalized);
            }
        }
        return std::string(scheme) + relative.generic_string();
    }

    StaticMeshAsset loadStaticMeshGlb(
        std::string identifier,
        const std::filesystem::path& path)
    {
        identifier = normalizeStaticMeshAssetIdentifier(identifier);
        if (path.extension() != ".glb")
        {
            assetError(identifier, "only .glb files are supported.");
        }
        const std::vector<std::byte> bytes = readFile(path, identifier);
        const GlbContents glb = parseGlb(bytes, identifier);
        const Json& document = glb.document;
        try
        {
            validateDocumentSubset(document, identifier);
            const Json& buffers = document.at("buffers");
            if (!buffers.is_array() || buffers.size() != 1
                || !buffers[0].is_object() || buffers[0].contains("uri")
                || !buffers[0].contains("byteLength")
                || !buffers[0]["byteLength"].is_number_unsigned()
                || buffers[0]["byteLength"].get<std::size_t>() > glb.binary.size())
            {
                assetError(identifier,
                    "GLB must contain exactly one embedded binary buffer.");
            }
            if (!document.contains("accessors")
                || !document.contains("bufferViews"))
            {
                assetError(identifier, "mesh has no accessor or bufferView data.");
            }

            const Json& primitives = document.at("meshes")[0].at("primitives");
            if (!primitives.is_array() || primitives.empty())
            {
                assetError(identifier, "mesh has no triangle geometry.");
            }

            StaticMeshAsset asset;
            asset.identifier = std::move(identifier);
            for (const Json& primitive : primitives)
            {
                if (!primitive.is_object())
                    assetError(asset.identifier, "mesh primitive is malformed.");
                if (primitive.value("mode", trianglePrimitiveMode)
                    != trianglePrimitiveMode)
                {
                    assetError(asset.identifier,
                        "mesh primitive mode is unsupported; export triangles.");
                }
                if (primitive.contains("targets"))
                {
                    assetError(asset.identifier, "morph targets are unsupported.");
                }
                if (primitive.contains("extensions"))
                {
                    assetError(asset.identifier,
                        "mesh primitive extensions are unsupported.");
                }
                if (!primitive.contains("indices")
                    || !primitive["indices"].is_number_unsigned()
                    || !primitive.contains("attributes")
                    || !primitive["attributes"].is_object())
                {
                    assetError(asset.identifier,
                        "triangle primitives require explicit indices and attributes.");
                }
                const Json& attributes = primitive["attributes"];
                if (!attributes.contains("POSITION")
                    || !attributes["POSITION"].is_number_unsigned())
                {
                    assetError(asset.identifier, "triangle primitive has no POSITION attribute.");
                }
                if (!attributes.contains("NORMAL")
                    || !attributes["NORMAL"].is_number_unsigned())
                {
                    assetError(asset.identifier,
                        "triangle primitive has no NORMAL attribute; export Blender normals.");
                }

                const AccessorView positions = accessorView(
                    document, glb.binary,
                    attributes["POSITION"].get<std::size_t>(),
                    "VEC3", componentFloat, asset.identifier);
                const AccessorView normals = accessorView(
                    document, glb.binary,
                    attributes["NORMAL"].get<std::size_t>(),
                    "VEC3", componentFloat, asset.identifier);
                const AccessorView indices = indexAccessorView(
                    document, glb.binary,
                    primitive["indices"].get<std::size_t>(), asset.identifier);
                if (positions.count != normals.count)
                {
                    assetError(asset.identifier,
                        "POSITION and NORMAL accessor counts differ.");
                }
                if (indices.count % 3 != 0)
                {
                    assetError(asset.identifier,
                        "triangle index count is not divisible by three.");
                }
                if (asset.vertices.size() > std::numeric_limits<std::uint32_t>::max()
                    - positions.count)
                {
                    assetError(asset.identifier, "vertex count exceeds 32-bit indices.");
                }
                const std::uint32_t baseVertex =
                    static_cast<std::uint32_t>(asset.vertices.size());
                asset.vertices.reserve(asset.vertices.size() + positions.count);
                for (std::size_t vertexIndex = 0;
                    vertexIndex < positions.count; ++vertexIndex)
                {
                    const glm::vec3 position = gltfVectorToQuantum(
                        readVec3(positions, vertexIndex));
                    glm::vec3 normal = gltfVectorToQuantum(
                        readVec3(normals, vertexIndex));
                    if (!finite(position) || !finite(normal)
                        || glm::length(normal) <= 1.0e-6F)
                    {
                        assetError(asset.identifier,
                            "mesh contains a non-finite or degenerate position/normal.");
                    }
                    normal = glm::normalize(normal);
                    asset.vertices.push_back({position, normal});
                }

                if (asset.triangleIndices.size()
                    > std::numeric_limits<std::uint32_t>::max() - indices.count)
                {
                    assetError(asset.identifier, "index count exceeds Vulkan's 32-bit draw range.");
                }
                const std::uint32_t firstIndex =
                    static_cast<std::uint32_t>(asset.triangleIndices.size());
                asset.triangleIndices.reserve(
                    asset.triangleIndices.size() + indices.count);
                for (std::size_t index = 0; index < indices.count; ++index)
                {
                    const std::uint32_t localIndex = readIndex(indices, index);
                    if (localIndex >= positions.count)
                    {
                        assetError(asset.identifier,
                            "triangle index is outside its primitive vertex stream.");
                    }
                    asset.triangleIndices.push_back(baseVertex + localIndex);
                }
                asset.submeshes.push_back({
                    firstIndex, static_cast<std::uint32_t>(indices.count)});
            }
            if (asset.vertices.empty() || asset.triangleIndices.empty())
            {
                assetError(asset.identifier, "mesh contains no triangle geometry.");
            }
            createEdges(asset);
            return asset;
        }
        catch (const Json::exception& exception)
        {
            assetError(identifier,
                "malformed glTF document: " + std::string(exception.what()));
        }
    }

    StaticMeshAssetCache::StaticMeshAssetCache(
        std::filesystem::path runtimeRoot)
        : runtimeRoot_(std::move(runtimeRoot))
    {
    }

    void StaticMeshAssetCache::setRuntimeRoot(
        std::filesystem::path runtimeRoot)
    {
        if (!assets_.empty() && runtimeRoot_ != runtimeRoot)
        {
            throw std::logic_error(
                "Static mesh asset root cannot change after assets are cached.");
        }
        runtimeRoot_ = std::move(runtimeRoot);
    }

    std::shared_ptr<const StaticMeshAsset> StaticMeshAssetCache::load(
        const std::string_view identifier)
    {
        const std::string normalized =
            normalizeStaticMeshAssetIdentifier(identifier);
        if (const auto existing = assets_.find(normalized);
            existing != assets_.end())
        {
            return existing->second;
        }

        StaticMeshAsset loaded;
        if (normalized == diagnosticHardwareAssetId)
        {
            loaded = diagnosticHardwareMesh();
        }
        else if (normalized.starts_with("builtin://"))
        {
            assetError(normalized, "unknown builtin asset identifier.");
        }
        else
        {
            if (runtimeRoot_.empty())
            {
                assetError(normalized, "runtime asset root is not configured.");
            }
            constexpr std::string_view assetsScheme = "assets://";
            loaded = loadStaticMeshGlb(
                normalized,
                runtimeRoot_ / "assets"
                    / std::filesystem::path(normalized.substr(assetsScheme.size())));
        }
        auto asset = std::make_shared<const StaticMeshAsset>(std::move(loaded));
        assets_.emplace(normalized, asset);
        return asset;
    }

    std::size_t StaticMeshAssetCache::size() const noexcept
    {
        return assets_.size();
    }

    bool StaticMeshAssetCache::invalidate(const std::string_view identifier)
    {
        return assets_.erase(normalizeStaticMeshAssetIdentifier(identifier)) > 0;
    }

    const std::filesystem::path& StaticMeshAssetCache::runtimeRoot() const noexcept
    {
        return runtimeRoot_;
    }

    StaticMeshGpuHandle StaticMeshGpuHandleCache::getOrUpload(
        const StaticMeshAsset& asset,
        const Upload& upload)
    {
        if (const auto found = handles_.find(asset.identifier);
            found != handles_.end())
        {
            return found->second;
        }
        if (!upload)
        {
            throw std::invalid_argument(
                "Static mesh GPU cache requires an upload operation.");
        }
        const StaticMeshGpuHandle handle = upload(asset);
        if (!handle)
        {
            throw std::runtime_error(
                "Static mesh GPU upload returned an invalid handle for '"
                + asset.identifier + "'.");
        }
        handles_.emplace(asset.identifier, handle);
        return handle;
    }

    std::size_t StaticMeshGpuHandleCache::size() const noexcept
    {
        return handles_.size();
    }

    std::optional<StaticMeshGpuHandle> StaticMeshGpuHandleCache::invalidate(
        const std::string_view identifier) noexcept
    {
        const auto found = handles_.find(std::string(identifier));
        if (found == handles_.end())
            return std::nullopt;
        const StaticMeshGpuHandle handle = found->second;
        handles_.erase(found);
        return handle;
    }

    void StaticMeshGpuHandleCache::clear() noexcept
    {
        handles_.clear();
    }
}
