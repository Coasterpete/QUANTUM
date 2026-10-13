#include <quantum/coaster/TrackConfiguration.hpp>
#include "TrackCatalogData.hpp"
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace quantum::coaster
{
    void TrackFamilyRegistry::registerType(TrackConfigurationDefinition definition)
    {
        if (definition.id.empty() || definition.displayName.empty()
            || definition.familyId.empty() || definition.category.empty())
            throw std::invalid_argument("Track catalog identity/category must not be empty.");
        if (types_.contains(definition.id))
            throw std::invalid_argument("Duplicate track type: " + definition.id);
        if (!definition.reserved) validateTrackStyle(definition.style);
        const auto key = definition.id;
        types_.emplace(key, std::move(definition));
    }
    const TrackConfigurationDefinition* TrackFamilyRegistry::find(std::string_view id) const noexcept
    {
        auto found = types_.find(id);
        return found == types_.end() ? nullptr : &found->second;
    }
    const std::map<std::string, TrackConfigurationDefinition, std::less<>>&
    TrackFamilyRegistry::types() const noexcept { return types_; }

    void TrackFamilyRegistry::loadJson(std::string_view text)
    {
        using Json = nlohmann::json;
        const auto document = Json::parse(text);
        if (document.at("schemaVersion") != 1 || !document.at("types").is_array())
            throw std::invalid_argument("Unsupported track catalog schema.");
        auto candidate = *this;
        auto pending = document.at("types");
        while (!pending.empty())
        {
            bool progress = false;
            for (auto it = pending.begin(); it != pending.end();)
            {
                const auto& item = *it;
                const auto baseId = item.value("baseId", std::string{});
                const auto* base = baseId.empty() ? nullptr : candidate.find(baseId);
                if (!baseId.empty() && !base) { ++it; continue; }
                TrackConfigurationDefinition def;
                if (base) def = *base;
                def.id = item.at("id").get<std::string>();
                def.displayName = item.at("displayName").get<std::string>();
                def.familyId = item.value("familyId", def.familyId);
                def.category = item.value("category", def.category);
                def.reference = item.value("reference", def.reference);
                def.baseId = baseId;
                def.reserved = item.value("reserved", false);
                if (!def.reserved)
                {
                    auto& style = def.style;
                    style.name = def.displayName;
                    style.geometryFamily = TrackGeometryFamily::Components;
                    style.familyId = def.familyId;
                    style.typeId = def.id;
                    style.spine.enabled = false;
                    style.repeatingHardware.clear();
                    style.railOffsets = {{-0.55, 0}, {0.55, 0}};
                    style.railCount = 2;
                    if (item.contains("construction"))
                    {
                        auto payload = base ? Json::parse(serializeTrackComponents(base->style)) : Json::object();
                        payload.merge_patch(item.at("construction"));
                        payload["schemaVersion"] = 1;
                        payload["familyId"] = def.familyId;
                        payload["typeId"] = def.id;
                        deserializeTrackComponents(style, payload.dump());
                    }
                    style.railOffsets = {{-0.55, 0}, {0.55, 0}};
                    style.railCount = 2;
                    style.railOffsets.clear();
                    for (const auto& component:style.components)
                        if (component.runningSurface)
                            style.railOffsets.push_back({component.offset.y,component.offset.z});
                    style.railCount=static_cast<std::uint32_t>(style.railOffsets.size());
                    if (item.contains("referenceOffsets"))
                    {
                        style.railOffsets.clear();
                        for (const auto& offset : item["referenceOffsets"])
                            style.railOffsets.push_back({offset[0], offset[1]});
                        style.railCount = static_cast<std::uint32_t>(style.railOffsets.size());
                    }
                }
                candidate.registerType(std::move(def));
                it = pending.erase(it);
                progress = true;
            }
            if (!progress) throw std::invalid_argument("Track catalog has a missing base or inheritance cycle.");
        }
        *this = std::move(candidate);
    }
    TrackFamilyRegistry& trackFamilyRegistry()
    {
        static TrackFamilyRegistry registry = [] {
            TrackFamilyRegistry result;
            TrackConfigurationDefinition legacy;
            legacy.id = modernSteelTrackConfigurationId;
            legacy.displayName = "Modern Steel";
            legacy.familyId = "legacy";
            legacy.category = "Legacy";
            legacy.reference = "Original QUANTUM Modern Steel";
            legacy.style = createModernSteelPreset();
            result.registerType(std::move(legacy));
            result.loadJson(detail::trackCatalogJson);
            return result;
        }();
        return registry;
    }
    const TrackConfigurationDefinition* findTrackConfiguration(std::string_view id)
    { return trackFamilyRegistry().find(id); }
    TrackStylePreset resolveTrackConfiguration(const TrackConfigurationDefinition& configuration)
    {
        const auto* def = findTrackConfiguration(configuration.id);
        if (!def || def->reserved)
            throw std::invalid_argument("Unavailable track configuration '" + configuration.id + "'.");
        validateTrackStyle(def->style);
        return def->style;
    }
    TrackStylePreset resolveTrackConfiguration(std::string_view id)
    {
        const auto* def = findTrackConfiguration(id);
        if (!def) throw std::invalid_argument("Unknown track configuration '" + std::string(id) + "'.");
        return resolveTrackConfiguration(*def);
    }
}
