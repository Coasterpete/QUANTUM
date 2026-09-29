#include <quantum/coaster/WoodenSupportGenerator.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace quantum::coaster
{
    namespace
    {
        struct FamilyRules
        {
            std::vector<double> lanes;
            double foundationSpread;
            bool adaptiveSpacing;
        };

        [[nodiscard]] FamilyRules rulesFor(const TimberSupportFamily family)
        {
            switch (family)
            {
            case TimberSupportFamily::TraditionalTimberBent:
                return {{-1.0, 1.0}, 1.0, false};
            case TimberSupportFamily::ModernTwisterTimber:
                return {{-1.0, 0.0, 1.0}, 1.15, true};
            case TimberSupportFamily::PrefabricatedTimberLattice:
                return {{-1.0, 0.0, 1.0}, 1.2, false};
            case TimberSupportFamily::HybridTimberLattice:
                return {{-1.0, 1.0}, 1.5, true};
            }
            throw std::invalid_argument("Unknown timber support family.");
        }

        struct Bent
        {
            // Levels run from foundation to track; each has one node per lane.
            std::vector<std::vector<SupportElementId>> levels;
            // Hybrid's first story has a raised ledger above the footings.
            std::vector<SupportElementId> lowerLedger;
        };

        [[nodiscard]] double nextSpacing(const WoodenSupportRunRecipe& recipe,
            const std::vector<TrackKinematicState>& states,
            const double station)
        {
            const double lookAhead = std::min(
                recipe.endStation, station + recipe.bentSpacing);
            const auto here = resolveSupportTrackAttachment(
                states, {station, 0.0, 0.0});
            const auto ahead = resolveSupportTrackAttachment(
                states, {lookAhead, 0.0, 0.0});
            const double headingChange = std::acos(std::clamp(
                glm::dot(here.frame.tangent, ahead.frame.tangent), -1.0, 1.0));
            const double bankChange = std::acos(std::clamp(
                glm::dot(here.frame.up, ahead.frame.up), -1.0, 1.0));
            const double threshold = recipe.family
                == TimberSupportFamily::ModernTwisterTimber ? 0.06 : 0.10;
            return recipe.bentSpacing
                * (std::max(headingChange, bankChange) > threshold ? 0.6 : 1.0);
        }
    }

    SupportStructureId generateWoodenSupportRun(
        AuthoredTrack& track,
        const WoodenSupportRunRecipe& recipe,
        const std::optional<SupportStructureId> replaceStructureId)
    {
        validateWoodenSupportRunRecipe(recipe);
        const FamilyRules rules = rulesFor(recipe.family);
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
        if (recipe.endStation - recipe.startStation < recipe.bentSpacing)
        {
            throw std::invalid_argument(
                "Wooden support range needs at least two bents.");
        }

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
        // Regeneration replaces topology, not appearance. Carrying the
        // authored tint forward is what lets a user regenerate a structure and
        // keep the timber color they chose.
        if (replacement != supports.structures.end())
        {
            structure.appearance = replacement->appearance;
            structure.foundationAppearance = replacement->foundationAppearance;
        }
        const SupportMemberProfile timber{
            SupportMemberProfileShape::Rectangular,
            {recipe.memberSize, recipe.memberSize}, 0.0};
        std::vector<glm::dvec3> nodePositions(1);
        const auto addNode = [&structure, &nodePositions](
            const glm::dvec3& position,
            const std::optional<TrackAttachment> attachment,
            const bool foundation)
        {
            const SupportElementId id = structure.nextElementId++;
            structure.nodes.push_back({id, position, attachment,
                foundation ? std::optional<Foundation>{Foundation{}} : std::nullopt});
            if (nodePositions.size() <= id)
            {
                nodePositions.resize(static_cast<std::size_t>(id) + 1);
            }
            nodePositions[id] = position;
            return id;
        };
        const auto addMember = [&structure, &timber, &nodePositions](
            const SupportElementId start, const SupportElementId end)
        {
            if (glm::length(nodePositions[end] - nodePositions[start]) <= 1e-6)
            {
                throw std::invalid_argument(
                    "Wooden support member has degenerate endpoints.");
            }
            structure.members.push_back({structure.nextElementId++, start, end, timber});
        };

        // Rider-frame top nodes follow banking; foundation lanes use the
        // horizontal tangent normal so they remain spread on the ground plane.
        const auto states = integrateAuthoredTrackKinematics(track, 0.25);
        std::optional<Bent> previous;
        std::size_t bentCount = 0;
        for (double station = recipe.startStation;
             station <= recipe.endStation + 1e-9;)
        {
            if (++bentCount > 10000)
            {
                throw std::invalid_argument("Wooden support run exceeds 10000 bents.");
            }
            const auto center = resolveSupportTrackAttachment(
                states, {station, 0.0, 0.0});
            glm::dvec3 lateral{
                -center.frame.tangent.y, center.frame.tangent.x, 0.0};
            const double lateralLength = glm::length(lateral);
            if (lateralLength <= 1e-6)
            {
                throw std::invalid_argument(
                    "Wooden bent needs a nonvertical track tangent.");
            }
            lateral /= lateralLength;
            const double halfWidth = recipe.bentWidth * 0.5;
            const glm::dvec3 baseCenter{
                center.position.x, center.position.y,
                recipe.foundationElevation};
            std::vector<glm::dvec3> bases;
            std::vector<glm::dvec3> tops;
            bases.reserve(rules.lanes.size());
            tops.reserve(rules.lanes.size());
            double maximumHeight = 0.0;
            double minimumHeight = std::numeric_limits<double>::max();
            for (const double lane : rules.lanes)
            {
                const auto top = resolveSupportTrackAttachment(states,
                    {station, lane * halfWidth, recipe.attachmentVerticalOffset});
                if (top.position.z <= recipe.foundationElevation + 1e-6)
                {
                    throw std::invalid_argument(
                        "Wooden bent upper points must be above the foundation plane.");
                }
                bases.push_back(baseCenter
                    + lane * halfWidth * rules.foundationSpread * lateral);
                tops.push_back(top.position);
                maximumHeight = std::max(maximumHeight,
                    top.position.z - recipe.foundationElevation);
                minimumHeight = std::min(minimumHeight,
                    top.position.z - recipe.foundationElevation);
            }
            const double storiesNeeded = std::ceil(maximumHeight / recipe.storyHeight);
            if (!std::isfinite(storiesNeeded) || storiesNeeded > 64.0)
            {
                throw std::invalid_argument(
                    "Wooden support run exceeds 64 framing stories.");
            }
            // Modern's lower longitudinal framing stays present on low runs.
            const auto storyCount = recipe.family
                == TimberSupportFamily::ModernTwisterTimber
                ? std::max<std::size_t>(2, static_cast<std::size_t>(storiesNeeded))
                : static_cast<std::size_t>(storiesNeeded);

            Bent bent;
            bent.levels.resize(storyCount + 1);
            for (std::size_t level = 0; level <= storyCount; ++level)
            {
                auto& nodes = bent.levels[level];
                const double fraction = static_cast<double>(level)
                    / static_cast<double>(storyCount);
                for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                {
                    const glm::dvec3 position =
                        bases[lane] + fraction * (tops[lane] - bases[lane]);
                    const bool attached = level == storyCount
                        && std::abs(rules.lanes[lane]) == 1.0;
                    nodes.push_back(addNode(position,
                        attached ? std::optional<TrackAttachment>{
                            {station, rules.lanes[lane] * halfWidth,
                                recipe.attachmentVerticalOffset}}
                            : std::nullopt,
                        level == 0));
                }
            }
            // Very short posts cannot fit a separate lower member and ledger.
            if (recipe.family == TimberSupportFamily::HybridTimberLattice
                && minimumHeight / static_cast<double>(storyCount) > 1e-4)
            {
                // The reference's first ledger is above the separate footings,
                // roughly a quarter of the way up the first framed story.
                const double fraction = 0.25 / static_cast<double>(storyCount);
                for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                {
                    bent.lowerLedger.push_back(addNode(
                        bases[lane] + fraction * (tops[lane] - bases[lane]),
                        std::nullopt, false));
                }
            }

            for (std::size_t level = 1; level <= storyCount; ++level)
            {
                const auto& lower = bent.levels[level - 1];
                const auto& upper = bent.levels[level];
                for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                {
                    if (level == 1 && !bent.lowerLedger.empty())
                    {
                        addMember(lower[lane], bent.lowerLedger[lane]);
                        addMember(bent.lowerLedger[lane], upper[lane]);
                    }
                    else
                    {
                        addMember(lower[lane], upper[lane]);
                    }
                }
                for (std::size_t lane = 1; lane < rules.lanes.size(); ++lane)
                {
                    addMember(upper[lane - 1], upper[lane]);
                }
                if (level == 1 && !bent.lowerLedger.empty())
                {
                    addMember(bent.lowerLedger.front(), bent.lowerLedger.back());
                }
                // The transverse bent is the repeated framed unit. Braces are
                // selective; neighboring ledgers supply most of the lattice.
                switch (recipe.family)
                {
                case TimberSupportFamily::TraditionalTimberBent:
                    addMember(lower[level % 2 == 0 ? 1 : 0],
                        upper[level % 2 == 0 ? 0 : 1]);
                    if (storyCount == 1)
                    {
                        // Keep the conventional M0 one-story bent geometry.
                        addMember(lower.back(), upper.front());
                    }
                    break;
                case TimberSupportFamily::ModernTwisterTimber:
                    if ((bentCount + level) % 2 == 0)
                        addMember(lower[0], upper[1]);
                    else
                        addMember(lower[2], upper[1]);
                    break;
                case TimberSupportFamily::PrefabricatedTimberLattice:
                    addMember(lower[0], upper[1]);
                    addMember(lower[2], upper[1]);
                    break;
                case TimberSupportFamily::HybridTimberLattice:
                {
                    const auto& braceFoot = level == 1
                        && !bent.lowerLedger.empty()
                            ? bent.lowerLedger : lower;
                    if (level % 2 == 1)
                        addMember(braceFoot.front(), upper.back());
                    else
                        addMember(braceFoot.back(), upper.front());
                    break;
                }
                }
            }

            if (previous)
            {
                if (!previous->lowerLedger.empty()
                    && !bent.lowerLedger.empty())
                {
                    for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                    {
                        addMember(previous->lowerLedger[lane],
                            bent.lowerLedger[lane]);
                    }
                }
                for (std::size_t level = 0; level < bent.levels.size(); ++level)
                {
                    const auto previousLevel = static_cast<std::size_t>(
                        std::round(static_cast<double>(level)
                            * static_cast<double>(previous->levels.size() - 1)
                            / static_cast<double>(storyCount)));
                    const auto& before = previous->levels[previousLevel];
                    const auto& after = bent.levels[level];
                    for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                    {
                        addMember(before[lane], after[lane]);
                    }
                }
                if (recipe.longitudinalBracing)
                {
                    for (std::size_t level = 1; level < bent.levels.size(); ++level)
                    {
                        const auto previousLower = static_cast<std::size_t>(
                            std::round(static_cast<double>(level - 1)
                                * static_cast<double>(previous->levels.size() - 1)
                                / static_cast<double>(storyCount)));
                        // Diagonals brace selected longitudinal bays, while
                        // most bays remain open between continuous ties.
                        if (recipe.family
                                == TimberSupportFamily::TraditionalTimberBent
                            && storyCount == 1)
                        {
                            // Preserve the M0 bracing of a low conventional run.
                            for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                            {
                                addMember(previous->levels[previousLower][lane],
                                    bent.levels[level][lane]);
                            }
                            continue;
                        }
                        const bool braceBay = recipe.family
                            == TimberSupportFamily::ModernTwisterTimber
                                ? bentCount % 2 == 0
                                : bentCount % 3 == 0;
                        if (braceBay)
                        {
                            if (recipe.family
                                == TimberSupportFamily::PrefabricatedTimberLattice)
                            {
                                // Braced bays repeat as complete tower modules;
                                // the other two bays remain open.
                                addMember(previous->levels[previousLower].front(),
                                    bent.levels[level].front());
                                addMember(previous->levels[previousLower].back(),
                                    bent.levels[level].back());
                            }
                            else
                            {
                                const std::size_t lane = (bentCount + level) % 2 == 0
                                    ? 0 : rules.lanes.size() - 1;
                                const auto& braceFoot = recipe.family
                                    == TimberSupportFamily::HybridTimberLattice
                                    && level == 1
                                    && !previous->lowerLedger.empty()
                                        ? previous->lowerLedger
                                        : previous->levels[previousLower];
                                addMember(braceFoot[lane],
                                    bent.levels[level][lane]);
                            }
                        }
                    }
                }
            }
            previous = std::move(bent);
            station += rules.adaptiveSpacing
                ? nextSpacing(recipe, states, station) : recipe.bentSpacing;
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
