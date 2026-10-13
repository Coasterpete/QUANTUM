#include <quantum/coaster/WoodenSupportGenerator.hpp>
#include <algorithm>
#include <array>
#include <stdexcept>

namespace quantum::coaster
{
    SupportStructureId generateSupportFrame(AuthoredTrack& track, SupportStyleFamily style,
        const SupportFrameRecipe& recipe, std::optional<SupportStructureId> replaceId)
    {
        validateSupportFrameRecipe(recipe);
        static_cast<void>(supportStyleName(style));
        const auto states = integrateAuthoredTrackKinematics(track, .25);
        auto collection = track.supports();
        auto old = std::ranges::find(collection.structures, replaceId.value_or(0), &SupportStructure::id);
        if (replaceId && (old == collection.structures.end() || !old->generatedFrame))
            throw std::invalid_argument("Regeneration requires a generated support frame.");
        if (old != collection.structures.end() && (old->nodes.size() < 4 || old->members.size() < 4))
            throw std::invalid_argument("Frame topology was deleted; create a new frame or Undo the deletion before regenerating.");
        SupportStructure structure;
        if (old != collection.structures.end()) structure = *old;
        else
        {
            structure.id = allocateSupportStructureId(collection);
            structure.name = std::string(supportStyleName(style)) + " frame " + std::to_string(structure.id);
        }
        structure.style = style;
        const auto interface = trackSupportAttachmentAt(track.trackStyle(),
            recipe.station * track.physicalSettings().metersPerCoordinateUnit) / track.physicalSettings().metersPerCoordinateUnit;
        const std::array<TrackAttachment, 2> attachments{{
            {recipe.station+interface.x, interface.y-recipe.width*.5, interface.z+recipe.verticalOffset},
            {recipe.station+interface.x, interface.y+recipe.width*.5, interface.z+recipe.verticalOffset}}};
        std::array<glm::dvec3, 4> positions;
        for (std::size_t lane = 0; lane < 2; ++lane)
        {
            positions[lane+2] = resolveSupportTrackAttachment(states, attachments[lane]).position;
            positions[lane] = positions[lane+2]; positions[lane].z = recipe.foundationElevation;
            if (positions[lane+2].z - recipe.foundationElevation <= 1e-6)
                throw std::invalid_argument("Frame foundations must be below both track attachments.");
        }
        std::array<SupportElementId, 4> ids;
        for (std::size_t i = 0; i < 4; ++i)
        {
            if (old == collection.structures.end())
                structure.nodes.push_back({allocateSupportElementId(structure), positions[i]});
            auto& node = structure.nodes[i];
            if (old != collection.structures.end() && node.id != i+1)
                throw std::invalid_argument("Frame topology was changed; create a new frame or Undo before regenerating.");
            ids[i] = node.id; node.position = positions[i];
            node.trackAttachment = i >= 2 ? std::optional{attachments[i-2]} : std::nullopt;
            node.foundation = i < 2 ? std::optional{Foundation{SupportFoundationKind::Footing, recipe.foundationSize}} : std::nullopt;
        }
        const std::array<std::pair<unsigned,unsigned>, 4> endpoints{{{0,2},{1,3},{2,3},{0,3}}};
        const std::array<SupportMemberRole,4> roles{SupportMemberRole::PrimaryPost, SupportMemberRole::PrimaryPost,
            SupportMemberRole::LedgerCap, SupportMemberRole::Brace};
        const std::array<SupportMemberProfile,4> profiles{recipe.post,recipe.post,recipe.beam,recipe.brace};
        const auto oldRecipe = structure.generatedFrame;
        for (std::size_t i = 0; i < 4; ++i)
        {
            if (old == collection.structures.end())
                structure.members.push_back({allocateSupportElementId(structure), ids[endpoints[i].first], ids[endpoints[i].second], profiles[i], roles[i]});
            auto& m = structure.members[i];
            // These four generated members retain their original local IDs.
            // Deleted/replaced topology must never be mistaken for the recipe.
            if (oldRecipe && (m.id != i+5 || ids[endpoints[i].first] != m.startNodeId
                || ids[endpoints[i].second] != m.endNodeId))
                throw std::invalid_argument("Frame topology was changed; create a new frame or Undo before regenerating.");
            if (oldRecipe)
            {
                const auto previous = i < 2 ? oldRecipe->post : i == 2 ? oldRecipe->beam : oldRecipe->brace;
                if (m.useFrameProfile && m.profile == previous) m.profile = profiles[i];
            }
            if (old == collection.structures.end())
            {
                m.useFrameProfile = true;
                if (i < 2)
                {
                    m.startConnection = SupportMemberEndConnection{SupportMemberEndTreatment::Base};
                    m.endConnection = SupportMemberEndConnection{SupportMemberEndTreatment::Saddle};
                }
                else if (i == 2) m.startConnection = SupportMemberEndConnection{SupportMemberEndTreatment::Plate};
            }
        }
        structure.generatedFrame = recipe;
        const auto id = structure.id;
        if (old == collection.structures.end()) collection.structures.push_back(std::move(structure));
        else *old = std::move(structure);
        track.setSupports(collection);
        return id;
    }
}
