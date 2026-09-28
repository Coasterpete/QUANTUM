#include <quantum/coaster/WoodenSupportGenerator.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace quantum::coaster
{
    SupportStructureId generateWoodenSupportRun(
        AuthoredTrack& track,
        const WoodenSupportRunRecipe& recipe,
        const std::optional<SupportStructureId> replaceStructureId)
    {
        validateWoodenSupportRunRecipe(recipe);
        double trackLength = 0.0;
        for (std::size_t index = 0; index < track.sectionCount(); ++index)
        {
            trackLength += sectionLength(track.section(index));
        }
        if (recipe.endStation > trackLength
            || (track.layoutMode() == LayoutMode::Circuit
                && recipe.endStation >= trackLength))
        {
            throw std::invalid_argument(
                "Wooden support range exceeds the authored track attachment domain.");
        }
        const double span = recipe.endStation - recipe.startStation;
        if (span < recipe.bentSpacing)
        {
            throw std::invalid_argument(
                "Wooden support range needs at least two bents.");
        }
        const double countValue = std::floor(span / recipe.bentSpacing) + 1.0;
        if (countValue > 10000.0)
        {
            throw std::invalid_argument("Wooden support run exceeds 10000 bents.");
        }
        const auto bentCount = static_cast<std::size_t>(countValue);

        SupportCollection supports = track.supports();
        auto replacement = supports.structures.end();
        if (replaceStructureId)
        {
            replacement = std::find_if(supports.structures.begin(),
                supports.structures.end(), [replaceStructureId](const SupportStructure& value)
                {
                    return value.id == *replaceStructureId;
                });
            if (replacement == supports.structures.end()
                || !replacement->generatedWoodenRun)
            {
                throw std::invalid_argument(
                    "Regeneration requires a generated wooden support structure.");
            }
        }

        SupportStructure structure;
        structure.id = replacement == supports.structures.end()
            ? allocateSupportStructureId(supports) : replacement->id;
        structure.name = "Wooden Run " + std::to_string(structure.id);
        structure.generatedWoodenRun = recipe;
        const SupportMemberProfile timber{
            SupportMemberProfileShape::Rectangular,
            {recipe.memberSize, recipe.memberSize}, 0.0};
        const auto addNode = [&structure](const glm::dvec3& position,
            const std::optional<TrackAttachment> attachment, const bool foundation)
        {
            const SupportElementId id = structure.nextElementId++;
            structure.nodes.push_back({id, position, attachment,
                foundation ? std::optional<Foundation>{Foundation{}} : std::nullopt});
            return id;
        };
        constexpr double minimumMemberLength = 1e-6;
        const auto addMember = [&structure, &timber](
            const SupportElementId start, const SupportElementId end)
        {
            const auto nodePosition = [&structure](const SupportElementId id)
            {
                const auto node = std::find_if(structure.nodes.begin(),
                    structure.nodes.end(), [id](const SupportNode& value)
                    {
                        return value.id == id;
                    });
                return node->position;
            };
            const auto a = nodePosition(start);
            const auto b = nodePosition(end);
            if (glm::length(b - a) <= minimumMemberLength)
            {
                throw std::invalid_argument(
                    "Wooden support member has degenerate endpoints.");
            }
            structure.members.push_back({structure.nextElementId++, start, end, timber});
        };

        // The canonical rider frame supplies banked upper attachments. The
        // horizontal plan-view lateral keeps the two foundations separated
        // even when the track is steeply banked.
        const auto states = integrateAuthoredTrackKinematics(track, 0.25);
        struct Bent { SupportElementId baseLeft, baseRight, topLeft, topRight; };
        std::optional<Bent> previous;
        constexpr double minimumHorizontalTangent = 1e-6;
        constexpr double minimumFoundationClearance = 1e-6;
        for (std::size_t index = 0; index < bentCount; ++index)
        {
            const double station = recipe.startStation
                + static_cast<double>(index) * recipe.bentSpacing;
            const auto center = resolveSupportTrackAttachment(
                states, {station, 0.0, 0.0});
            glm::dvec3 horizontalLateral{
                -center.frame.tangent.y, center.frame.tangent.x, 0.0};
            const double lateralLength = glm::length(horizontalLateral);
            if (lateralLength <= minimumHorizontalTangent)
            {
                throw std::invalid_argument(
                    "Wooden bent needs a nonvertical track tangent.");
            }
            horizontalLateral /= lateralLength;
            const double halfWidth = recipe.bentWidth * 0.5;
            const auto left = resolveSupportTrackAttachment(states,
                {station, -halfWidth, recipe.attachmentVerticalOffset});
            const auto right = resolveSupportTrackAttachment(states,
                {station, halfWidth, recipe.attachmentVerticalOffset});
            const glm::dvec3 baseCenter{
                center.position.x, center.position.y,
                recipe.foundationElevation};
            const glm::dvec3 baseLeft = baseCenter - halfWidth * horizontalLateral;
            const glm::dvec3 baseRight = baseCenter + halfWidth * horizontalLateral;
            if (left.position.z
                    <= recipe.foundationElevation + minimumFoundationClearance
                || right.position.z
                    <= recipe.foundationElevation + minimumFoundationClearance)
            {
                throw std::invalid_argument(
                    "Wooden bent upper points must be above the foundation plane.");
            }
            Bent bent{
                addNode(baseLeft, std::nullopt, true),
                addNode(baseRight, std::nullopt, true),
                addNode(left.position,
                    TrackAttachment{station, -halfWidth,
                        recipe.attachmentVerticalOffset}, false),
                addNode(right.position,
                    TrackAttachment{station, halfWidth,
                        recipe.attachmentVerticalOffset}, false)};
            addMember(bent.baseLeft, bent.topLeft);
            addMember(bent.baseRight, bent.topRight);
            addMember(bent.topLeft, bent.topRight);
            addMember(bent.baseLeft, bent.topRight);
            addMember(bent.baseRight, bent.topLeft);
            if (previous)
            {
                addMember(previous->topLeft, bent.topLeft);
                addMember(previous->topRight, bent.topRight);
                addMember(previous->baseLeft, bent.baseLeft);
                addMember(previous->baseRight, bent.baseRight);
                if (recipe.longitudinalBracing)
                {
                    addMember(previous->baseLeft, bent.topLeft);
                    addMember(previous->baseRight, bent.topRight);
                }
            }
            previous = bent;
        }

        const SupportStructureId generatedId = structure.id;
        if (replacement == supports.structures.end())
        {
            supports.structures.push_back(std::move(structure));
        }
        else
        {
            *replacement = std::move(structure);
        }
        track.setSupports(supports);
        return generatedId;
    }
}
