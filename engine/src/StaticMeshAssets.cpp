#include <quantum/renderer/StaticMeshAssets.hpp>
#include <quantum/coaster/TrackConfiguration.hpp>
#include <quantum/coaster/TrackComponentDiagnostics.hpp>
#include <quantum/engine/Logging.hpp>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <functional>
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

    [[noreturn]] void assetError(std::string_view identifier,
        std::string_view message);

    [[nodiscard]] float linearToSrgb(const float value) noexcept
    {
        return value <= 0.0031308F ? 12.92F * value
            : 1.055F * std::pow(value, 1.0F / 2.4F) - 0.055F;
    }

    [[nodiscard]] std::optional<quantum::coaster::TrackMaterial>
        primitiveMaterial(const Json& document, const Json& primitive,
            const std::string_view identifier)
    {
        if (!primitive.contains("material"))
            return std::nullopt;
        const auto materialIndex = primitive["material"].get<std::size_t>();
        if (!document.contains("materials")
            || !document["materials"].is_array()
            || materialIndex >= document["materials"].size())
            assetError(identifier, "primitive references an invalid material.");
        const Json& material = document["materials"][materialIndex];
        if (!material.is_object())
            assetError(identifier, "material is malformed.");
        if (material.value("alphaMode",std::string{"OPAQUE"})!="OPAQUE")
            assetError(identifier,"transparent/masked static component materials are unsupported.");
        if (material.contains("extensions"))
            assetError(identifier,"static component material extensions are unsupported.");
        if (material.contains("emissiveFactor"))
            for (const auto& factor:material.at("emissiveFactor"))
                if (factor.get<double>()!=0) assetError(identifier,"emissive static component materials are unsupported.");
        if (material.contains("normalTexture") || material.contains("occlusionTexture") || material.contains("emissiveTexture"))
            assetError(identifier,"material textures are unsupported by the static component pipeline.");
        const Json& pbr = material.contains("pbrMetallicRoughness")
            ? material["pbrMetallicRoughness"] : Json::object();
        if (!pbr.is_object())
            assetError(identifier, "pbrMetallicRoughness is malformed.");
        if (pbr.contains("baseColorTexture") || pbr.contains("metallicRoughnessTexture"))
            assetError(identifier,"PBR textures are unsupported by the static component pipeline.");
        quantum::coaster::TrackMaterial result;
        result.baseColor = {1.0F, 1.0F, 1.0F, 1.0F};
        result.metallic = pbr.value("metallicFactor", 1.0F);
        result.roughness = pbr.value("roughnessFactor", 1.0F);
        if (pbr.contains("baseColorFactor"))
        {
            const Json& color = pbr["baseColorFactor"];
            if (!color.is_array() || color.size() != 4)
                assetError(identifier, "baseColorFactor must have four components.");
            for (int channel = 0; channel < 3; ++channel)
                result.baseColor[channel] = linearToSrgb(
                    color[channel].get<float>());
            result.baseColor.a = color[3].get<float>();
        }
        if (!std::isfinite(result.metallic)
            || !std::isfinite(result.roughness)
            || result.metallic < 0.0F || result.metallic > 1.0F
            || result.roughness < 0.0F || result.roughness > 1.0F)
            assetError(identifier, "metallic-roughness factors must be in [0, 1].");
        for (int channel = 0; channel < 4; ++channel)
            if (!std::isfinite(result.baseColor[channel])
                || result.baseColor[channel] < 0.0F
                || result.baseColor[channel] > 1.0F)
                assetError(identifier, "baseColorFactor must be in [0, 1].");
        return result;
    }

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
        if (document.contains("extensionsRequired") && !document["extensionsRequired"].empty())
            assetError(identifier, "required glTF extensions are unsupported.");
        if (document.contains("nodes") && !document["nodes"].is_array())
            assetError(identifier, "nodes must be an array.");
    }

    struct MeshOccurrence { std::size_t mesh; glm::dmat4 transform; };
    std::vector<MeshOccurrence> meshOccurrences(const Json& document, std::string_view identifier)
    {
        std::vector<MeshOccurrence> result;
        const auto& meshes=document.at("meshes");
        if (!document.contains("nodes"))
        {
            for (std::size_t i=0;i<meshes.size();++i) result.push_back({i,glm::dmat4{1}});
            return result;
        }
        const auto& nodes=document.at("nodes");
        std::vector<bool> child(nodes.size(),false), visiting(nodes.size(),false), visited(nodes.size(),false);
        for (const auto& node:nodes)
            for (const auto& index:node.value("children",Json::array()))
            {
                auto i=index.get<std::size_t>();
                if (i>=nodes.size()) assetError(identifier,"child node index is invalid.");
                if (child[i]) assetError(identifier,"node has multiple parents.");
                child[i]=true;
            }
        std::function<void(std::size_t,const glm::dmat4&)> visit;
        visit=[&](std::size_t index,const glm::dmat4& parent)
        {
            if (index>=nodes.size() || visiting[index]) assetError(identifier,"invalid or cyclic scene hierarchy.");
            if (visited[index]) assetError(identifier,"node occurs more than once in the active scene.");
            visiting[index]=true; visited[index]=true;
            const auto& node=nodes[index];
            if (!node.is_object()) assetError(identifier,"node is malformed.");
            for (const char* field:{"skin","camera","extensions"})
                if (node.contains(field)) assetError(identifier,"unsupported node feature: "+std::string(field));
            glm::dmat4 local{1};
            if (node.contains("matrix"))
            {
                if (node.contains("translation") || node.contains("rotation") || node.contains("scale"))
                    assetError(identifier,"node cannot have both matrix and TRS.");
                const auto& matrix=node.at("matrix");
                if (!matrix.is_array() || matrix.size()!=16) assetError(identifier,"matrix must have 16 entries.");
                for (int k=0;k<16;++k) local[k/4][k%4]=matrix[k].get<double>();
            }
            else
            {
                const auto vec=[&](const char* key,glm::dvec3 fallback) {
                    if (!node.contains(key)) return fallback;
                    const auto& v=node.at(key);
                    if (!v.is_array() || v.size()!=3) assetError(identifier,"TRS vector must have three entries.");
                    return glm::dvec3{v[0].get<double>(),v[1].get<double>(),v[2].get<double>()};
                };
                glm::dquat rotation{1,0,0,0};
                if (node.contains("rotation"))
                {
                    const auto& q=node.at("rotation");
                    if (!q.is_array() || q.size()!=4) assetError(identifier,"rotation must have four entries.");
                    rotation={q[3].get<double>(),q[0].get<double>(),q[1].get<double>(),q[2].get<double>()};
                    if (!std::isfinite(glm::length(rotation)) || std::abs(glm::length(rotation)-1)>1e-5)
                        assetError(identifier,"rotation quaternion must be finite and unit length.");
                }
                local=glm::translate(local,vec("translation",glm::dvec3{0}))*glm::mat4_cast(rotation);
                local=glm::scale(local,vec("scale",glm::dvec3{1}));
            }
            const auto world=parent*local;
            for (int c=0;c<4;++c) for (int r=0;r<4;++r)
                if (!std::isfinite(world[c][r])) assetError(identifier,"node transform is nonfinite.");
            if (std::abs(glm::determinant(glm::dmat3{world}))<1e-12
                || world[0][3]!=0 || world[1][3]!=0 || world[2][3]!=0 || world[3][3]!=1)
                assetError(identifier,"node transform must be nonsingular and affine.");
            if (node.contains("mesh"))
            {
                const auto mesh=node.at("mesh").get<std::size_t>();
                if (mesh>=meshes.size()) assetError(identifier,"node mesh index is invalid.");
                result.push_back({mesh,world});
            }
            for (const auto& next:node.value("children",Json::array())) visit(next.get<std::size_t>(),world);
            visiting[index]=false;
        };
        if (document.contains("scenes"))
        {
            const auto& scenes=document.at("scenes");
            const auto selected=document.value("scene",std::size_t{0});
            if (!scenes.is_array() || selected>=scenes.size()) assetError(identifier,"default scene index is invalid.");
            for (const auto& root:scenes[selected].at("nodes")) visit(root.get<std::size_t>(),glm::dmat4{1});
        }
        else
        {
            for (std::size_t i=0;i<nodes.size();++i) if (!child[i]) visit(i,glm::dmat4{1});
            if (std::ranges::any_of(visited,[](bool value){return !value;}))
                assetError(identifier,"scene hierarchy has no valid root.");
        }
        return result;
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

        const std::size_t components = expectedType == "VEC3" ? 3 : expectedType == "VEC2" ? 2 : 1;
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

            StaticMeshAsset asset;
            asset.identifier = identifier;
            for (const auto& occurrence : meshOccurrences(document, asset.identifier))
            {
            const auto& primitives=document.at("meshes")[occurrence.mesh].at("primitives");
            if (!primitives.is_array() || primitives.empty()) assetError(asset.identifier,"mesh has no triangle geometry.");
            const auto normalTransform=glm::inverseTranspose(glm::dmat3{occurrence.transform});
            const bool mirrored=glm::determinant(glm::dmat3{occurrence.transform})<0;
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
                std::optional<AccessorView> uvs;
                if (attributes.contains("TEXCOORD_0"))
                {
                    uvs=accessorView(document,glb.binary,attributes["TEXCOORD_0"].get<std::size_t>(),"VEC2",componentFloat,asset.identifier);
                    if (uvs->count!=positions.count) assetError(asset.identifier,"UV and position counts differ.");
                }
                if (attributes.contains("COLOR_0")) assetError(asset.identifier,"vertex colors are unsupported by static component presentation.");
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
                        glm::vec3{occurrence.transform * glm::dvec4{readVec3(positions, vertexIndex),1}});
                    glm::vec3 normal = gltfVectorToQuantum(
                        glm::vec3{normalTransform * glm::dvec3{readVec3(normals, vertexIndex)}});
                    if (!finite(position) || !finite(normal)
                        || glm::length(normal) <= 1.0e-6F)
                    {
                        assetError(asset.identifier,
                            "mesh contains a non-finite or degenerate position/normal.");
                    }
                    normal = glm::normalize(normal);
                    glm::vec2 uv{0};
                    if (uvs)
                    {
                        std::memcpy(&uv.x,uvs->data+vertexIndex*uvs->stride,sizeof(float));
                        std::memcpy(&uv.y,uvs->data+vertexIndex*uvs->stride+sizeof(float),sizeof(float));
                        if (!std::isfinite(uv.x) || !std::isfinite(uv.y)) assetError(asset.identifier,"UV is nonfinite.");
                    }
                    asset.vertices.push_back({position, normal, uv});
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
                    const auto sourceIndex=mirrored ? (index/3)*3+(index%3==1 ? 2 : index%3==2 ? 1 : 0) : index;
                    const std::uint32_t localIndex = readIndex(indices, sourceIndex);
                    if (localIndex >= positions.count)
                    {
                        assetError(asset.identifier,
                            "triangle index is outside its primitive vertex stream.");
                    }
                    asset.triangleIndices.push_back(baseVertex + localIndex);
                }
                asset.submeshes.push_back({
                    firstIndex, static_cast<std::uint32_t>(indices.count),
                    primitiveMaterial(document, primitive, asset.identifier)});
            }
            }
            if (asset.vertices.empty() || asset.triangleIndices.empty())
            {
                assetError(asset.identifier, "mesh contains no triangle geometry.");
            }
            asset.minimumPosition=asset.maximumPosition=asset.vertices.front().position;
            for (const auto& vertex:asset.vertices)
            { asset.minimumPosition=glm::min(asset.minimumPosition,vertex.position); asset.maximumPosition=glm::max(asset.maximumPosition,vertex.position); }
            createEdges(asset);
            return asset;
        }
        catch (const Json::exception& exception)
        {
            assetError(identifier,
                "malformed glTF document: " + std::string(exception.what()));
        }
    }

    namespace
    {
        struct ComponentAssetState
        {
            StaticMeshAssetCache assets;
            std::unordered_map<std::string,std::shared_ptr<const coaster::TrackComponentMesh>> meshes;
            std::unordered_map<std::string,HardwareAssetLoadStatus> statuses;
        };
        thread_local std::shared_ptr<ComponentAssetState> componentAssets;
    }
    void configureTrackComponentAssets(const std::filesystem::path& root)
    {
        // Discover additional versioned definitions beside the bundled catalog.
        // Registration is atomic per file; malformed extensions leave the saved
        // component snapshots usable and receive an actionable diagnostic.
        const auto directory=root/"assets/track/catalog";
        static std::set<std::filesystem::path> loadedCatalogs;
        if (std::filesystem::is_directory(directory))
        {
            std::vector<std::filesystem::path> catalogs;
            for (const auto& entry:std::filesystem::directory_iterator(directory))
                if (entry.is_regular_file() && entry.path().extension()==".json"
                    && entry.path().filename()!="numbered-families.json") catalogs.push_back(entry.path());
            std::ranges::sort(catalogs);
            for (const auto& path:catalogs)
            {
                if (loadedCatalogs.contains(path)) continue;
                std::ifstream file(path);
                if (!file) throw std::runtime_error("Cannot read track catalog: "+path.string());
                const std::string text{std::istreambuf_iterator<char>{file},{}};
                try { coaster::trackFamilyRegistry().loadJson(text); loadedCatalogs.insert(path); }
                catch (const std::exception& error) {
                    logging::logMessagef(logging::LogLevel::Warning,"TRACK","Catalog %s rejected: %s",path.string().c_str(),error.what());
                }
            }
        }
        auto state=std::make_shared<ComponentAssetState>();
        state->assets.setRuntimeRoot(root);
        componentAssets=state;
        coaster::setTrackComponentDiagnosticSink([state](std::string_view id,std::string_view detail) {
            const auto key=normalizeStaticMeshAssetIdentifier(id);
            state->statuses[key]={key,detail.empty() ? HardwareAssetLoadState::Loaded : HardwareAssetLoadState::UnsupportedGlb,
                !detail.empty(),detail.empty() ? "Loaded curve-following component." : std::string{detail}};
        });
        coaster::setTrackComponentMeshResolver([state](std::string_view id) -> std::shared_ptr<const coaster::TrackComponentMesh> {
            const auto key=normalizeStaticMeshAssetIdentifier(id);
            if (const auto found=state->meshes.find(key); found!=state->meshes.end()) return found->second;
            try
            {
                const auto source=state->assets.load(key);
                auto mesh=std::make_shared<coaster::TrackComponentMesh>();
                for (const auto& v:source->vertices) mesh->mesh.vertices.push_back({v.position,v.normal,v.uv});
                mesh->mesh.triangleIndices=source->triangleIndices;
                mesh->mesh.edgeIndices=source->edgeIndices;
                for (const auto& sub:source->submeshes)
                {
                    auto material=static_cast<std::uint32_t>(mesh->materials.size());
                    mesh->materials.push_back(sub.material.value_or(coaster::TrackMaterial{}));
                    mesh->mesh.submeshes.push_back({sub.firstIndex,sub.indexCount,material,0});
                }
                state->statuses[key]={key,HardwareAssetLoadState::Loaded,false,"Loaded static component; node transforms applied."};
                state->meshes[key]=mesh;
                return mesh;
            }
            catch (const std::runtime_error& error)
            {
                state->statuses[key]={key,classifyStaticMeshLoadFailure(error.what()),true,error.what()};
                state->meshes[key]=nullptr;
                return nullptr;
            }
        });
    }
    std::optional<HardwareAssetLoadStatus> trackComponentAssetStatus(std::string_view id)
    {
        if (!componentAssets) return std::nullopt;
        const auto found=componentAssets->statuses.find(std::string{id});
        if (found==componentAssets->statuses.end()) return std::nullopt;
        return found->second;
    }
    void reloadTrackComponentAsset(std::string_view id)
    {
        if (!componentAssets) return;
        const auto key=normalizeStaticMeshAssetIdentifier(id);
        componentAssets->assets.invalidate(key);
        componentAssets->meshes.erase(key); componentAssets->statuses.erase(key);
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
