#include <quantum/coaster/Supports.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace quantum::coaster
{
    const char* supportMaterialName(SupportMaterialKind material)
    {
        switch (material)
        {
        case SupportMaterialKind::Timber: return "Timber";
        case SupportMaterialKind::PaintedSteel: return "PaintedSteel";
        case SupportMaterialKind::GalvanizedSteel: return "GalvanizedSteel";
        case SupportMaterialKind::Concrete: return "Concrete";
        }
        throw std::invalid_argument("Unknown support material.");
    }
    SupportMaterialKind supportMaterialFromName(std::string_view name)
    {
        for (unsigned i = 0; i <= 3; ++i)
            if (name == supportMaterialName(static_cast<SupportMaterialKind>(i)))
                return static_cast<SupportMaterialKind>(i);
        throw std::invalid_argument("Unknown support material: " + std::string(name));
    }
    SupportSurface defaultSupportSurface(SupportMaterialKind material)
    {
        static_cast<void>(supportMaterialName(material));
        SupportSurface surface;
        surface.material = material;
        switch (material)
        {
        case SupportMaterialKind::Timber: break;
        case SupportMaterialKind::PaintedSteel:
            surface.baseColorTint = {.23F, .38F, .48F};
            surface.roughness = .42F; surface.normalStrength = .15F;
            surface.repeatMeters = {1, 1}; break;
        case SupportMaterialKind::GalvanizedSteel:
            surface.baseColorTint = {.68F, .71F, .73F};
            surface.roughness = .48F; surface.metallic = 1;
            surface.normalStrength = .2F; surface.repeatMeters = {.5F, .5F}; break;
        case SupportMaterialKind::Concrete:
            surface.baseColorTint = {.62F, .62F, .60F};
            surface.roughness = .92F; surface.normalStrength = .35F;
            surface.repeatMeters = {1, 1}; break;
        }
        return surface;
    }
    void validateSupportSurface(const SupportSurface& s)
    {
        static_cast<void>(supportMaterialName(s.material));
        for (float c : {s.baseColorTint.x, s.baseColorTint.y, s.baseColorTint.z})
            if (!std::isfinite(c) || c < 0 || c > 1)
                throw std::invalid_argument("Support material tint must be finite and in [0,1].");
        if (!std::isfinite(s.roughness) || s.roughness < .045F || s.roughness > 1
            || !std::isfinite(s.metallic) || s.metallic < 0 || s.metallic > 1
            || !std::isfinite(s.normalStrength) || s.normalStrength < 0 || s.normalStrength > 4
            || !std::isfinite(s.repeatMeters.x) || !std::isfinite(s.repeatMeters.y)
            || s.repeatMeters.x <= 0 || s.repeatMeters.y <= 0)
            throw std::invalid_argument("Invalid support material roughness, metallic, normal or SI repeat scale.");
    }
    std::string serializeSupportSurface(const SupportSurface& s)
    {
        validateSupportSurface(s);
        return nlohmann::json{{"material", supportMaterialName(s.material)},
            {"tint", {s.baseColorTint.x, s.baseColorTint.y, s.baseColorTint.z}},
            {"roughness", s.roughness}, {"metallic", s.metallic},
            {"normalStrength", s.normalStrength},
            {"repeatMeters", {s.repeatMeters.x, s.repeatMeters.y}}}.dump();
    }
    SupportSurface deserializeSupportSurface(std::string_view text)
    {
        const auto j = nlohmann::json::parse(text);
        if (!j.is_object()) throw std::invalid_argument("Support surface must be an object.");
        for (const auto& [key, value] : j.items())
            if (key != "material" && key != "tint" && key != "roughness"
                && key != "metallic" && key != "normalStrength" && key != "repeatMeters")
                throw std::invalid_argument("Unknown support surface field: " + key);
        auto s = defaultSupportSurface(supportMaterialFromName(j.at("material").get<std::string>()));
        if (j.contains("tint"))
        {
            const auto& a = j.at("tint");
            if (!a.is_array() || a.size() != 3) throw std::invalid_argument("Support tint needs three channels.");
            s.baseColorTint = {a[0].get<float>(), a[1].get<float>(), a[2].get<float>()};
        }
        if (j.contains("repeatMeters"))
        {
            const auto& a = j.at("repeatMeters");
            if (!a.is_array() || a.size() != 2) throw std::invalid_argument("Support repeat needs two distances.");
            s.repeatMeters = {a[0].get<float>(), a[1].get<float>()};
        }
        s.roughness = j.value("roughness", s.roughness);
        s.metallic = j.value("metallic", s.metallic);
        s.normalStrength = j.value("normalStrength", s.normalStrength);
        validateSupportSurface(s);
        return s;
    }
    const char* supportStyleName(SupportStyleFamily style)
    {
        switch (style)
        {
        case SupportStyleFamily::TraditionalTimber: return "TraditionalTimber";
        case SupportStyleFamily::TubularSteel: return "TubularSteel";
        case SupportStyleFamily::BoxSteel: return "BoxSteel";
        case SupportStyleFamily::ISectionSteel: return "ISectionSteel";
        case SupportStyleFamily::ConcretePier: return "ConcretePier";
        case SupportStyleFamily::Custom: return "Custom";
        }
        throw std::invalid_argument("Unknown support style.");
    }
    SupportStyleFamily supportStyleFromName(std::string_view name)
    {
        for (unsigned i = 0; i <= 5; ++i)
            if (name == supportStyleName(static_cast<SupportStyleFamily>(i)))
                return static_cast<SupportStyleFamily>(i);
        throw std::invalid_argument("Unknown support style: " + std::string(name));
    }
    SupportMemberProfile supportStyleProfile(SupportStyleFamily style, SupportMemberRole role)
    {
        static_cast<void>(supportStyleName(style));
        const bool brace = role == SupportMemberRole::Brace;
        const bool beam = role == SupportMemberRole::LedgerCap || role == SupportMemberRole::TrackSupport;
        switch (style)
        {
        case SupportStyleFamily::TraditionalTimber: return timberProfileForRole(role, .3);
        case SupportStyleFamily::TubularSteel: return {SupportMemberProfileShape::Circular,
            glm::dvec2{brace ? .18 : .5}, brace ? .012 : .025};
        case SupportStyleFamily::BoxSteel: return {SupportMemberProfileShape::Hss,
            {brace ? .15 : .3, brace ? .15 : (beam ? .45 : .3)}, .015};
        case SupportStyleFamily::ISectionSteel: return {SupportMemberProfileShape::ISection,
            {brace ? .18 : .3, brace ? .25 : .5}, .02};
        case SupportStyleFamily::ConcretePier: return {SupportMemberProfileShape::Rectangular,
            {brace ? .3 : .8, brace ? .3 : .8}, 0};
        case SupportStyleFamily::Custom: return defaultSupportMemberProfile();
        }
        throw std::invalid_argument("Unknown support style.");
    }
    SupportMemberProfile resolveSupportProfile(const SupportStructure& s, const SupportMember& m)
    {
        return m.useStyleProfile ? supportStyleProfile(s.style, m.role) : m.profile;
    }
    SupportSurface resolveSupportSurface(const SupportStructure& s, const SupportMember& m, double scale)
    {
        if (m.surface) return *m.surface;
        if (s.surface) return *s.surface;
        auto kind = s.style == SupportStyleFamily::ConcretePier ? SupportMaterialKind::Concrete
            : s.style == SupportStyleFamily::TraditionalTimber || s.style == SupportStyleFamily::Custom
            ? SupportMaterialKind::Timber : SupportMaterialKind::PaintedSteel;
        auto surface = defaultSupportSurface(kind);
        if (kind == SupportMaterialKind::Timber && s.appearance)
        {
            surface.baseColorTint = s.appearance->baseColorTint;
            surface.roughness = std::clamp(.85F * s.appearance->roughnessMultiplier, .045F, 1.0F);
            surface.normalStrength = std::clamp(s.appearance->normalStrength, 0.0F, 4.0F);
            // Legacy repeat is Core units. New surfaces use SI metres.
            surface.repeatMeters *= static_cast<float>(s.appearance->textureScale * scale);
        }
        return surface;
    }
    void validateSupportFrameRecipe(const SupportFrameRecipe& r)
    {
        if (!std::isfinite(r.station) || r.station < 0 || !std::isfinite(r.width) || r.width <= 0
            || !std::isfinite(r.foundationElevation) || !std::isfinite(r.verticalOffset))
            throw std::invalid_argument("Invalid support frame station, width or elevation.");
        for (double d : {r.foundationSize.x, r.foundationSize.y, r.foundationSize.z})
            if (!std::isfinite(d) || d <= 0) throw std::invalid_argument("Frame foundation size must be positive.");
        validateSupportMemberProfile(r.post); validateSupportMemberProfile(r.beam); validateSupportMemberProfile(r.brace);
    }
}
