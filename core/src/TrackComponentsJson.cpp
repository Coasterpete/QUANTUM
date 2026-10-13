#include <quantum/coaster/TrackStyle.hpp>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <algorithm>
#include <limits>

namespace quantum::coaster
{
    namespace
    {
        using Json = nlohmann::json;
        void fields(const Json& object, std::initializer_list<std::string_view> allowed)
        {
            if (!object.is_object()) throw std::invalid_argument("Track component payload must be an object.");
            for (auto it=object.begin(); it!=object.end(); ++it)
                if (std::find(allowed.begin(),allowed.end(),it.key())==allowed.end())
                    throw std::invalid_argument("Unknown track component field: "+it.key());
        }
        std::uint64_t identifier(const Json& value)
        {
            if (!value.is_number_integer() || (value.is_number_integer() && !value.is_number_unsigned() && value.get<std::int64_t>()<0))
                throw std::invalid_argument("Construction IDs must be nonnegative integers.");
            return value.get<std::uint64_t>();
        }

        template<class V> Json vectorJson(const V& v) { return Json::array({v.x, v.y, v.z}); }
        glm::dvec3 vector3(const Json& j)
        {
            if (!j.is_array() || j.size() != 3) throw std::invalid_argument("Component vector must have three coordinates.");
            return {j[0].get<double>(), j[1].get<double>(), j[2].get<double>()};
        }
        Json materialJson(const TrackMaterial& m)
        { return {{"color", {m.baseColor.r, m.baseColor.g, m.baseColor.b, m.baseColor.a}},
            {"metallic", m.metallic}, {"roughness", m.roughness}}; }
        TrackMaterial material(const Json& j)
        {
            fields(j,{"color","metallic","roughness"});
            TrackMaterial m;
            if (j.contains("color"))
            {
                auto c = j.at("color");
                if (!c.is_array() || c.size() != 4) throw std::invalid_argument("Component color must have four channels.");
                m.baseColor = {c[0], c[1], c[2], c[3]};
            }
            m.metallic = j.value("metallic", m.metallic);
            m.roughness = j.value("roughness", m.roughness);
            return m;
        }
        Json componentsJson(const std::vector<TrackComponent>& components)
        {
            Json result = Json::array();
            for (const auto& c : components)
                result.push_back({{"id", c.id}, {"displayName", c.displayName}, {"materialSlot", c.materialSlot},
                    {"provider", c.provider == TrackComponentProvider::Tube ? "tube" : c.provider == TrackComponentProvider::Box ? "box" : c.provider == TrackComponentProvider::Member ? "member" : "beam"},
                    {"placement", c.placement == TrackComponentPlacement::Continuous ? "continuous" : c.placement == TrackComponentPlacement::Repeated ? "repeated" : "junction"},
                    {"enabled", c.enabled}, {"runningSurface", c.runningSurface}, {"required", c.required}, {"offset", vectorJson(c.offset)},
                    {"endOffset", vectorJson(c.endOffset)}, {"dimensions", vectorJson(c.dimensions)},
                    {"spacing", c.spacing}, {"phase", c.phase}, {"radialSegments", c.radialSegments},
                    {"material", materialJson(c.material)}, {"asset", c.asset.path}, {"placeholder", c.asset.placeholder},
                    {"useAssetMaterials", c.useAssetMaterials}, {"assetLength", c.assetLength}, {"assetScale", vectorJson(c.assetScale)}});
            return result;
        }
        std::vector<TrackComponent> components(const Json& array)
        {
            if (!array.is_array()) throw std::invalid_argument("Track components must be an array.");
            std::vector<TrackComponent> result;
            for (const auto& j : array)
            {
                fields(j,{"id","displayName","materialSlot","provider","placement","enabled","runningSurface","required",
                    "offset","endOffset","dimensions","spacing","phase","radialSegments","material","asset","placeholder","assetLength","assetScale","useAssetMaterials"});
                TrackComponent c;
                c.id = j.at("id").get<std::string>();
                c.displayName = j.value("displayName", c.id);
                c.materialSlot = j.value("materialSlot", std::string{"structure"});
                auto provider = j.at("provider").get<std::string>();
                if (provider == "tube") c.provider = TrackComponentProvider::Tube;
                else if (provider == "box") c.provider = TrackComponentProvider::Box;
                else if (provider == "member") c.provider = TrackComponentProvider::Member;
                else if (provider == "beam") c.provider = TrackComponentProvider::Beam;
                else throw std::invalid_argument("Unsupported track component provider: " + provider);
                auto placement = j.value("placement", std::string{"continuous"});
                if (placement == "continuous") c.placement = TrackComponentPlacement::Continuous;
                else if (placement == "repeated") c.placement = TrackComponentPlacement::Repeated;
                else if (placement == "junction") c.placement = TrackComponentPlacement::Junction;
                else throw std::invalid_argument("Unsupported component placement: " + placement);
                c.enabled = j.value("enabled", true);
                c.runningSurface = j.value("runningSurface", false);
                c.required = j.value("required", false);
                c.offset = vector3(j.at("offset"));
                if (j.contains("endOffset")) c.endOffset = vector3(j["endOffset"]);
                c.dimensions = vector3(j.at("dimensions"));
                c.spacing = j.value("spacing", 1.0);
                c.phase = j.value("phase", 0.0);
                c.radialSegments = j.value("radialSegments", 12u);
                if (j.contains("material")) c.material = material(j["material"]);
                c.asset.path = j.value("asset", std::string{});
                c.asset.placeholder = j.value("placeholder", false);
                c.useAssetMaterials = j.value("useAssetMaterials", true);
                c.assetLength = j.value("assetLength", 1.0);
                if (j.contains("assetScale")) c.assetScale = vector3(j["assetScale"]);
                result.push_back(std::move(c));
            }
            return result;
        }
    }
    std::string serializeTrackComponents(const TrackStylePreset& style)
    {
        Json variants = Json::array(), intervals = Json::array();
        for (const auto& v : style.variants)
            variants.push_back({{"id", v.id}, {"displayName", v.displayName},
                {"components", componentsJson(v.components)}, {"supportAttachment", vectorJson(v.supportAttachment)}});
        for (const auto& i : style.constructionIntervals)
            intervals.push_back({{"id", i.id}, {"start", i.start}, {"end", i.end},
                {"typeId", i.typeId}, {"variantId", i.variantId}, {"components", componentsJson(i.components)}});
        return Json{{"schemaVersion", 1}, {"familyId", style.familyId}, {"typeId", style.typeId},
            {"variantId", style.variantId}, {"components", componentsJson(style.components)},
            {"variants", variants}, {"intervals", intervals}, {"nextIntervalId", style.nextConstructionIntervalId},
            {"supportAttachment", vectorJson(style.supportAttachment)}}.dump();
    }
    void deserializeTrackComponents(TrackStylePreset& style, std::string_view text)
    {
        auto j = Json::parse(text);
        fields(j,{"schemaVersion","familyId","typeId","variantId","components","variants","intervals","nextIntervalId","supportAttachment"});
        if (j.value("schemaVersion", 1) != 1) throw std::invalid_argument("Unsupported track component schema.");
        auto candidate = style;
        candidate.familyId = j.value("familyId", std::string{});
        candidate.typeId = j.value("typeId", std::string{});
        candidate.variantId = j.value("variantId", std::string{});
        candidate.components = components(j.at("components"));
        candidate.variants.clear();
        for (const auto& v : j.value("variants", Json::array()))
        {
            fields(v,{"id","displayName","components","supportAttachment"});
            candidate.variants.push_back({v.at("id"), v.at("displayName"), components(v.at("components")),
                vector3(v.at("supportAttachment"))});
        }
        candidate.constructionIntervals.clear();
        for (const auto& i : j.value("intervals", Json::array()))
        {
            fields(i,{"id","start","end","typeId","variantId","components"});
            candidate.constructionIntervals.push_back({identifier(i.at("id")), i.at("start"), i.at("end"),
                i.value("typeId", candidate.typeId), i.at("variantId"), components(i.value("components", Json::array()))});
        }
        candidate.nextConstructionIntervalId = j.contains("nextIntervalId") ? identifier(j.at("nextIntervalId")) : 1;
        if (candidate.nextConstructionIntervalId==0 || candidate.nextConstructionIntervalId==std::numeric_limits<std::uint64_t>::max())
            throw std::invalid_argument("Construction interval allocator is invalid or exhausted.");
        if (j.contains("supportAttachment")) candidate.supportAttachment = vector3(j["supportAttachment"]);
        validateTrackStyle(candidate);
        style = std::move(candidate);
    }
}
