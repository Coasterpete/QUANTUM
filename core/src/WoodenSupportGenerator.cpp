#include <quantum/coaster/WoodenSupportGenerator.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <unordered_map>
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
                return {{-1.0, 1.0}, 1.0, true};
            }
            throw std::invalid_argument("Unknown timber support family.");
        }

        struct Bent
        {
            // Levels run from foundation to terminal cap (Hybrid shoulder);
            // each row has one node per primary post lane.
            std::vector<std::vector<SupportElementId>> levels;
            // Interior rows use actual common elevations, not height fractions.
            // The final row is the terminal cap and is matched separately.
            std::vector<double> elevations;
            // Hybrid keeps banked attachments separate from the lower frame.
            std::vector<SupportElementId> trackInterface;
            // Hybrid's first story has a raised ledger above the footings.
            std::vector<SupportElementId> lowerLedger;
            // Connected towers also split these post chains at local tie heights.
            // Such points do not become transverse story rows.
            std::vector<std::vector<SupportElementId>> postNodes;
        };

        [[nodiscard]] std::vector<std::pair<std::size_t, std::size_t>>
            correspondingLevels(const Bent& before, const Bent& after)
        {
            std::vector<std::pair<std::size_t, std::size_t>> result{{0, 0}};
            std::size_t a = 1;
            std::size_t b = 1;
            while (a + 1 < before.levels.size() && b + 1 < after.levels.size())
            {
                const double difference = before.elevations[a] - after.elevations[b];
                if (std::abs(difference) <= 1e-6)
                {
                    result.emplace_back(a++, b++);
                }
                else if (difference < 0.0)
                {
                    ++a;
                }
                else
                {
                    ++b;
                }
            }
            // An unmatched ledger ends at its own bent. The terminal caps
            // connect once across the resulting stepped upper bay; braces
            // use these same matched panel boundaries, never rounded rows.
            result.emplace_back(before.levels.size() - 1, after.levels.size() - 1);
            return result;
        }

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

        [[nodiscard]] bool connectedHybridAssembly(const WoodenSupportRunRecipe& recipe,
            const std::vector<TrackKinematicState>& states)
        {
            if (recipe.family != TimberSupportFamily::HybridTimberLattice)
                return false;
            if (recipe.hybridArchetype != HybridFramingArchetype::Automatic)
                return recipe.hybridArchetype == HybridFramingArchetype::ConnectedTowers;
            // Automatic selects the assembly when any sampled tower needs more
            // than one lower-frame story. This is a replaceable QUANTUM heuristic.
            std::size_t count = 0;
            for (double station = recipe.startStation;
                 station <= recipe.endStation + 1e-9 && count++ < 10000;
                 station += nextSpacing(recipe, states, station))
            {
                const double left = resolveSupportTrackAttachment(states,
                    {station, -recipe.bentWidth * 0.5, recipe.attachmentVerticalOffset}).position.z;
                const double right = resolveSupportTrackAttachment(states,
                    {station, recipe.bentWidth * 0.5, recipe.attachmentVerticalOffset}).position.z;
                const double height = std::min(left, right) - recipe.foundationElevation;
                const double shoulderHeight = height - 0.25 * std::min(height, recipe.storyHeight);
                if (shoulderHeight > recipe.storyHeight + 1e-6)
                    return true;
            }
            return false;
        }

        [[nodiscard]] HybridTransversePanelChoice transversePanelChoice(
            const WoodenSupportRunRecipe& recipe, const std::size_t tower, const std::size_t panel)
        {
            const auto choice = std::find_if(recipe.hybridTransversePanels.begin(),
                recipe.hybridTransversePanels.end(), [tower, panel](const auto& value)
                { return value.towerIndex == tower && value.panelIndex == panel; });
            return choice == recipe.hybridTransversePanels.end()
                ? HybridTransversePanelChoice{} : *choice;
        }

        [[nodiscard]] HybridLongitudinalBracing longitudinalPanelChoice(
            const WoodenSupportRunRecipe& recipe, const std::size_t bay,
            const std::size_t panel, const std::size_t lane)
        {
            const auto choice = std::find_if(recipe.hybridLongitudinalPanels.begin(),
                recipe.hybridLongitudinalPanels.end(), [bay, panel, lane](const auto& value)
                { return value.bayIndex == bay && value.panelIndex == panel && value.laneIndex == lane; });
            return choice == recipe.hybridLongitudinalPanels.end()
                ? HybridLongitudinalBracing::Open : choice->bracing;
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
        // Hybrid section proportions are QUANTUM approximations, not measured
        // lumber dimensions. TrackSupport remains available in the role model,
        // but this simple bent generator does not establish a stringer function.
        const auto hybridProfileForRole =
            [&](const SupportMemberRole role) -> SupportMemberProfile
        {
            const double s = recipe.memberSize;
            if (!std::isfinite(s) || s <= 0.0)
            {
                throw std::invalid_argument(
                    "Hybrid timber member size must be finite and positive.");
            }
            switch (role)
            {
            case SupportMemberRole::PrimaryPost:
                return {SupportMemberProfileShape::Rectangular, {s, s}, 0.0};
            case SupportMemberRole::LedgerCap:
                return {SupportMemberProfileShape::Rectangular,
                    {s * 0.95, s * 0.75}, 0.0};
            case SupportMemberRole::TrackSupport:
                return {SupportMemberProfileShape::Rectangular,
                    {s * 0.80, s * 0.62}, 0.0};
            case SupportMemberRole::LongitudinalTie:
                return {SupportMemberProfileShape::Rectangular,
                    {s * 0.70, s * 0.55}, 0.0};
            case SupportMemberRole::Brace:
                return {SupportMemberProfileShape::Rectangular,
                    {s * 0.55, s * 0.45}, 0.0};
            case SupportMemberRole::Unspecified:
                break;
            }
            throw std::invalid_argument(
                "Hybrid support members require an explicit structural role.");
        };
        std::vector<glm::dvec3> nodePositions(1);
        std::unordered_map<SupportElementId, SupportMemberMountingFace> postSideFaces;
        std::unordered_map<SupportElementId, SupportMemberMountingFace> transverseBraceFaces;
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
        // Hybrid has its own section proportions. Other families use the
        // shared role table; neither table changes during integration cleanup.
        const auto profileForRole =
            [&](const SupportMemberRole role) -> SupportMemberProfile
        {
            if (recipe.family == TimberSupportFamily::HybridTimberLattice)
            {
                return hybridProfileForRole(role);
            }
            if (role == SupportMemberRole::Unspecified)
                throw std::invalid_argument(
                    "Wooden support members require an explicit structural role.");
            return timberProfileForRole(role, recipe.memberSize);
        };
        // Only Hybrid authors a structural plane and bent-frame reference.
        // The other families retain Generic orientation without a reference.
        const auto addMember = [&structure, &nodePositions, &profileForRole](
            const SupportElementId start,
            const SupportElementId end,
            const SupportMemberRole role,
            const SupportMemberOrientation orientation =
                SupportMemberOrientation::Generic,
            SupportMemberOrientationReference orientationReference =
                std::nullopt)
        {
            if (glm::length(nodePositions[end] - nodePositions[start]) <= 1e-6)
            {
                throw std::invalid_argument(
                    "Wooden support member has degenerate endpoints.");
            }
            validateSupportMemberOrientation(orientation);
            if (orientationReference.has_value())
            {
                const glm::dvec3 axis = glm::normalize(nodePositions[end] - nodePositions[start]);
                const glm::dvec3 perpendicular = *orientationReference
                    - axis * glm::dot(*orientationReference, axis);
                if (glm::length(perpendicular) <= 1e-6)
                {
                    throw std::invalid_argument(
                        "Wooden support member has degenerate structural orientation geometry.");
                }
                orientationReference =
                    normalizeSupportMemberOrientationReference(
                        perpendicular);
            }
            structure.members.push_back({structure.nextElementId++, start, end,
                profileForRole(role), role, orientation, orientationReference});
        };

        // Rider-frame top nodes follow banking; foundation lanes use the
        // horizontal tangent normal so they remain spread on the ground plane.
        const auto states = integrateAuthoredTrackKinematics(track, 0.25);
        const bool connectedTowers = connectedHybridAssembly(recipe, states);
        std::vector<Bent> towers;
        std::vector<std::pair<Bent, HybridOuterSupportChoice>> outerSupportTowers;
        std::optional<Bent> previous;
        const bool hybridBent = recipe.family == TimberSupportFamily::HybridTimberLattice;
        const auto direction = [&nodePositions](const SupportElementId start,
            const SupportElementId end) -> glm::dvec3
        {
            return nodePositions[end] - nodePositions[start];
        };
        const auto across = [&direction](const std::vector<SupportElementId>& row)
        {
            return direction(row.front(), row.back());
        };
        const auto reference = [hybridBent](const glm::dvec3& geometry)
            -> SupportMemberOrientationReference
        {
            return hybridBent ? SupportMemberOrientationReference{geometry} : std::nullopt;
        };
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
            // Reserve a short upper connection below the lowest attachment.
            // The quarter-story clearance is a procedural approximation, not
            // a hardware dimension. Lower timber rows stay horizontal and
            // remain in the unbanked, upright bent plane.
            const double frameworkHeight = hybridBent
                ? minimumHeight - 0.25 * std::min(minimumHeight, recipe.storyHeight)
                : minimumHeight;
            std::vector<glm::dvec3> frameworkTops = tops;
            if (hybridBent)
            {
                for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                {
                    frameworkTops[lane] = baseCenter
                        + rules.lanes[lane] * halfWidth * lateral;
                    frameworkTops[lane].z += frameworkHeight;
                }
            }
            Bent bent;
            bent.elevations.push_back(recipe.foundationElevation);
            for (double height = recipe.storyHeight;
                 height < frameworkHeight - 1e-6; height += recipe.storyHeight)
            {
                bent.elevations.push_back(recipe.foundationElevation + height);
            }
            // Preserve Modern's existing low-run tier rule for its later review.
            if (recipe.family == TimberSupportFamily::ModernTwisterTimber
                && bent.elevations.size() == 1)
            {
                bent.elevations.push_back(recipe.foundationElevation + minimumHeight * 0.5);
            }
            bent.elevations.push_back(recipe.foundationElevation + frameworkHeight);
            const std::size_t storyCount = bent.elevations.size() - 1;
            bent.levels.resize(storyCount + 1);
            for (std::size_t level = 0; level <= storyCount; ++level)
            {
                auto& nodes = bent.levels[level];
                for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                {
                    const double fraction = level == storyCount ? 1.0
                        : (bent.elevations[level] - recipe.foundationElevation)
                            / (frameworkTops[lane].z - recipe.foundationElevation);
                    const glm::dvec3 position =
                        bases[lane] + fraction * (frameworkTops[lane] - bases[lane]);
                    const bool attached = !hybridBent && level == storyCount
                        && std::abs(rules.lanes[lane]) == 1.0;
                    nodes.push_back(addNode(position,
                        attached ? std::optional<TrackAttachment>{
                            {station, rules.lanes[lane] * halfWidth,
                                recipe.attachmentVerticalOffset}}
                            : std::nullopt,
                        level == 0));
                }
            }
            if (hybridBent)
            {
                for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                {
                    bent.trackInterface.push_back(addNode(tops[lane],
                        TrackAttachment{station, rules.lanes[lane] * halfWidth,
                            recipe.attachmentVerticalOffset}, false));
                }
            }
            // Very short posts cannot fit a separate lower member and ledger.
            if (recipe.family == TimberSupportFamily::HybridTimberLattice
                && minimumHeight / static_cast<double>(storyCount) > 1e-4)
            {
                // The reference's first ledger is above the separate footings,
                // roughly a quarter of the way up the first framed story.
                for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                {
                    bent.lowerLedger.push_back(addNode(
                        bases[lane] + 0.25
                            * (nodePositions[bent.levels[1][lane]] - bases[lane]),
                        std::nullopt, false));
                }
            }

            if (hybridBent)
            {
                for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                {
                    const auto face = lane == 0 ? SupportMemberMountingFace::NegativeY
                        : SupportMemberMountingFace::PositiveY;
                    for (const auto& row : bent.levels) postSideFaces.emplace(row[lane], face);
                    postSideFaces.emplace(bent.trackInterface[lane], face);
                    if (!bent.lowerLedger.empty()) postSideFaces.emplace(bent.lowerLedger[lane], face);
                }
            }

            for (std::size_t level = 1; level <= storyCount; ++level)
            {
                const auto& lower = bent.levels[level - 1];
                const auto& upper = bent.levels[level];
                // Hybrid orientation tags name the bent/run plane each member
                // was framed in. Other families keep the Generic fallback, so
                // their call sites below are intentionally untagged.
                const bool hybrid =
                    recipe.family == TimberSupportFamily::HybridTimberLattice;
                const SupportMemberOrientation postOrientation = hybrid
                    ? SupportMemberOrientation::BentPost
                    : SupportMemberOrientation::Generic;
                const SupportMemberOrientation transverseOrientation = hybrid
                    ? SupportMemberOrientation::BentTransverse
                    : SupportMemberOrientation::Generic;
                // Posts use their actual bounding rows. Each ledger uses the
                // incident post at its start, defining a local triangle even
                // when the four panel corners are not coplanar.
                for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                {
                    if (level == 1 && !bent.lowerLedger.empty())
                    {
                        addMember(lower[lane], bent.lowerLedger[lane],
                            SupportMemberRole::PrimaryPost, postOrientation,
                            reference(across(lower) + across(bent.lowerLedger)));
                        addMember(bent.lowerLedger[lane], upper[lane],
                            SupportMemberRole::PrimaryPost, postOrientation,
                            reference(across(bent.lowerLedger) + across(upper)));
                    }
                    else
                    {
                        addMember(lower[lane], upper[lane],
                            SupportMemberRole::PrimaryPost, postOrientation,
                            reference(across(lower) + across(upper)));
                    }
                }
                for (std::size_t lane = 1; lane < rules.lanes.size(); ++lane)
                {
                    addMember(upper[lane - 1], upper[lane],
                        SupportMemberRole::LedgerCap, transverseOrientation,
                        reference(direction(lower[lane - 1], upper[lane - 1])));
                }
                if (level == 1 && !bent.lowerLedger.empty())
                {
                    addMember(bent.lowerLedger.front(), bent.lowerLedger.back(),
                        SupportMemberRole::LedgerCap, transverseOrientation,
                        reference(direction(lower.front(), bent.lowerLedger.front())));
                }
                // The transverse bent is the repeated framed unit. Braces are
                // selective; neighboring ledgers supply most of the lattice.
                switch (recipe.family)
                {
                case TimberSupportFamily::TraditionalTimberBent:
                    addMember(lower[level % 2 == 0 ? 1 : 0],
                        upper[level % 2 == 0 ? 0 : 1],
                        SupportMemberRole::Brace);
                    if (storyCount == 1)
                    {
                        // Keep the conventional M0 one-story bent geometry.
                        addMember(lower.back(), upper.front(),
                            SupportMemberRole::Brace);
                    }
                    break;
                case TimberSupportFamily::ModernTwisterTimber:
                    if ((bentCount + level) % 2 == 0)
                        addMember(lower[0], upper[1],
                            SupportMemberRole::Brace);
                    else
                        addMember(lower[2], upper[1],
                            SupportMemberRole::Brace);
                    break;
                case TimberSupportFamily::PrefabricatedTimberLattice:
                    addMember(lower[0], upper[1], SupportMemberRole::Brace);
                    addMember(lower[2], upper[1], SupportMemberRole::Brace);
                    break;
                case TimberSupportFamily::HybridTimberLattice:
                {
                    // The default repeats one single diagonal; an explicit
                    // local panel may select either direction on either face.
                    const auto choice = transversePanelChoice(recipe, bentCount - 1, level - 1);
                    const auto& braceFoot = level == 1
                        && !bent.lowerLedger.empty()
                            ? bent.lowerLedger : lower;
                    const bool forward = choice.direction == HybridDiagonalDirection::LowerFirstToUpperLast;
                    const auto start = forward ? braceFoot.front() : braceFoot.back();
                    const auto end = forward ? upper.back() : upper.front();
                    // The brace and its incident lower row define this
                    // member's plane; no whole-bent planarity is assumed.
                    addMember(start, end, SupportMemberRole::Brace,
                        SupportMemberOrientation::BentDiagonal,
                        reference(glm::cross(across(braceFoot), direction(start, end))));
                    transverseBraceFaces.emplace(structure.members.back().id, choice.face);
                    break;
                }
                }
            }

            if (hybridBent)
            {
                for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                {
                    addMember(bent.levels.back()[lane], bent.trackInterface[lane],
                        SupportMemberRole::PrimaryPost, SupportMemberOrientation::BentPost,
                        reference(across(bent.levels.back()) + across(bent.trackInterface)));
                }
                addMember(bent.trackInterface.front(), bent.trackInterface.back(),
                    SupportMemberRole::LedgerCap, SupportMemberOrientation::BentTransverse,
                    reference(direction(bent.levels.back().front(), bent.trackInterface.front())));
            }

            if (previous && !connectedTowers)
            {
                const SupportMemberOrientation runOrientation =
                    recipe.family == TimberSupportFamily::HybridTimberLattice
                    ? SupportMemberOrientation::RunLongitudinal
                    : SupportMemberOrientation::Generic;
                // Ties use the actual post incident to their start endpoint.
                // Braces use that side's post and the diagonal itself: a
                // member-specific triangle for a potentially twisted bay.
                if (!previous->lowerLedger.empty()
                    && !bent.lowerLedger.empty())
                {
                    for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                    {
                        addMember(previous->lowerLedger[lane],
                            bent.lowerLedger[lane],
                            SupportMemberRole::LongitudinalTie,
                            runOrientation,
                            reference(direction(previous->levels.front()[lane],
                                previous->lowerLedger[lane])));
                    }
                }
                if (hybridBent)
                {
                    for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                    {
                        addMember(previous->trackInterface[lane], bent.trackInterface[lane],
                            SupportMemberRole::LongitudinalTie, runOrientation,
                            reference(direction(previous->levels.back()[lane],
                                previous->trackInterface[lane])));
                    }
                }
                const auto correspondence = correspondingLevels(*previous, bent);
                for (const auto [previousLevel, level] : correspondence)
                {
                    // Hybrid footings are discrete: no grade-level timber ties
                    // footing to footing. The reference shows longitudinal
                    // ties at the raised ledger and story levels with open
                    // bays between, not a ground beam burying the footings.
                    if (recipe.family
                            == TimberSupportFamily::HybridTimberLattice
                        && level == 0)
                    {
                        continue;
                    }
                    const auto& before = previous->levels[previousLevel];
                    const auto& after = bent.levels[level];
                    // Hybrid's shoulder and interface connections are ties.
                    // Row height alone does not establish a stringer function.
                    // Other families retain their roles for their later review.
                    const SupportMemberRole tieRole = !hybridBent && level + 1
                            == bent.levels.size()
                        ? SupportMemberRole::TrackSupport
                        : SupportMemberRole::LongitudinalTie;
                    for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                    {
                        addMember(
                            before[lane], after[lane], tieRole, runOrientation,
                            hybridBent ? reference(direction(
                                previous->levels[previousLevel - 1][lane], before[lane]))
                                : std::nullopt);
                    }
                }
                if (recipe.longitudinalBracing)
                {
                    for (std::size_t panel = 1; panel < correspondence.size(); ++panel)
                    {
                        const auto previousLower = correspondence[panel - 1].first;
                        const auto previousUpper = correspondence[panel].first;
                        const auto level = correspondence[panel].second;
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
                                    bent.levels[level][lane],
                                    SupportMemberRole::Brace);
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
                                    bent.levels[level].front(),
                                    SupportMemberRole::Brace);
                                addMember(previous->levels[previousLower].back(),
                                    bent.levels[level].back(),
                                    SupportMemberRole::Brace);
                            }
                            else
                            {
                                const std::size_t lane = (bentCount + level) % 2 == 0
                                    ? 0 : rules.lanes.size() - 1;
                                const auto& braceFoot = recipe.family
                                    == TimberSupportFamily::HybridTimberLattice
                                    && panel == 1
                                    && !previous->lowerLedger.empty()
                                        ? previous->lowerLedger
                                        : previous->levels[previousLower];
                                // Hybrid longitudinal braces name the run
                                // plane; other families keep Generic.
                                const SupportMemberOrientation braceOrientation =
                                    recipe.family
                                        == TimberSupportFamily::
                                            HybridTimberLattice
                                    ? SupportMemberOrientation::RunDiagonal
                                    : SupportMemberOrientation::Generic;
                                addMember(braceFoot[lane],
                                    bent.levels[level][lane],
                                    SupportMemberRole::Brace, braceOrientation,
                                    reference(glm::cross(
                                        direction(braceFoot[lane], previous->levels[previousUpper][lane]),
                                        direction(braceFoot[lane], bent.levels[level][lane]))));
                            }
                        }
                    }
                }
            }
            if (hybridBent)
            {
                const auto choice = std::find_if(recipe.hybridOuterSupports.begin(),
                    recipe.hybridOuterSupports.end(), [bentCount](const auto& value)
                    { return value.towerIndex == bentCount - 1; });
                if (choice != recipe.hybridOuterSupports.end()
                    && choice->sides != HybridOuterSupportSides::None)
                    outerSupportTowers.emplace_back(bent, *choice);
            }
            if (connectedTowers)
            {
                bent.postNodes.resize(rules.lanes.size());
                for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                {
                    auto& chain = bent.postNodes[lane];
                    chain.push_back(bent.levels.front()[lane]);
                    if (!bent.lowerLedger.empty())
                        chain.push_back(bent.lowerLedger[lane]);
                    for (std::size_t level = 1; level < bent.levels.size(); ++level)
                        chain.push_back(bent.levels[level][lane]);
                }
                towers.push_back(std::move(bent));
            }
            else
                previous = std::move(bent);
            station += rules.adaptiveSpacing
                ? nextSpacing(recipe, states, station) : recipe.bentSpacing;
        }

        if (connectedTowers)
        {
            // Insert real structural nodes and split the incident primary post.
            // Reuse existing points at equal elevations, including points made
            // for the other neighboring bay. No transverse ledger is added.
            const auto postPoint = [&](Bent& tower, const std::size_t lane,
                const double elevation) -> SupportElementId
            {
                auto& chain = tower.postNodes[lane];
                for (std::size_t index = 0; index < chain.size(); ++index)
                {
                    const auto upper = chain[index];
                    if (std::abs(nodePositions[upper].z - elevation) <= 1e-6)
                        return upper;
                    if (index == 0 || nodePositions[upper].z < elevation)
                        continue;
                    const auto lower = chain[index - 1];
                    const double fraction = (elevation - nodePositions[lower].z)
                        / (nodePositions[upper].z - nodePositions[lower].z);
                    const auto point = addNode(nodePositions[lower]
                        + fraction * direction(lower, upper), std::nullopt, false);
                    postSideFaces.emplace(point, postSideFaces.at(lower));
                    const auto member = std::find_if(structure.members.begin(), structure.members.end(),
                        [lower, upper](const SupportMember& value)
                        {
                            return value.role == SupportMemberRole::PrimaryPost
                                && value.startNodeId == lower && value.endNodeId == upper;
                        });
                    if (member == structure.members.end())
                        throw std::logic_error("Hybrid tie connection has no incident primary post.");
                    const auto orientationReference = member->orientationReference;
                    member->endNodeId = point;
                    addMember(point, upper, SupportMemberRole::PrimaryPost,
                        SupportMemberOrientation::BentPost, orientationReference);
                    chain.insert(chain.begin() + static_cast<std::ptrdiff_t>(index), point);
                    return point;
                }
                throw std::logic_error("Hybrid tie connection lies outside its tower post.");
            };
            for (std::size_t bay = 0; bay + 1 < towers.size(); ++bay)
            {
                auto& before = towers[bay];
                auto& after = towers[bay + 1];
                // Adjacent bays stagger their local horizontals. The 0.35-story
                // offset is a visual QUANTUM approximation of the reference,
                // not a measured joint offset or manufacturer framing rule.
                const double offset = bay % 2 == 0 ? 0.0 : 0.35 * recipe.storyHeight;
                const double ceiling = std::min(before.elevations.back(), after.elevations.back());
                std::vector<double> connections;
                if (offset == 0.0 && !before.lowerLedger.empty() && !after.lowerLedger.empty())
                {
                    const double elevation = std::max(nodePositions[before.lowerLedger.front()].z,
                        nodePositions[after.lowerLedger.front()].z);
                    if (elevation <= ceiling + 1e-6)
                        connections.push_back(elevation);
                }
                // Terminal caps stay local to their tower. Using them as another
                // staggered row can bunch ties beside a short terminal story.
                for (std::size_t level = 1; level + 1 < before.elevations.size(); ++level)
                {
                    const double elevation = before.elevations[level] - offset;
                    if (elevation > recipe.foundationElevation + 1e-6
                        && elevation <= ceiling + 1e-6)
                        connections.push_back(elevation);
                }
                std::sort(connections.begin(), connections.end());
                connections.erase(std::unique(connections.begin(), connections.end(),
                    [](const double a, const double b) { return std::abs(a - b) <= 1e-6; }),
                    connections.end());
                for (const double elevation : connections)
                {
                    for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                    {
                        const auto start = postPoint(before, lane, elevation);
                        const auto end = postPoint(after, lane, elevation);
                        const auto& chain = before.postNodes[lane];
                        const auto point = std::find(chain.begin(), chain.end(), start);
                        addMember(start, end, SupportMemberRole::LongitudinalTie,
                            SupportMemberOrientation::RunLongitudinal,
                            reference(direction(*(point - 1), start)));
                    }
                }
                // Only consecutive actual tie rows bound a run-side panel.
                // No cap/interface or unmatched tower story is used to fill it.
                for (std::size_t panel = 0; panel + 1 < connections.size(); ++panel)
                    for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                    {
                        if (longitudinalPanelChoice(recipe, bay, panel, lane)
                            != HybridLongitudinalBracing::SingleDiagonal) continue;
                        const auto start = postPoint(before, lane, connections[panel]);
                        const auto upper = postPoint(before, lane, connections[panel + 1]);
                        const auto end = postPoint(after, lane, connections[panel + 1]);
                        addMember(start, end, SupportMemberRole::Brace,
                            SupportMemberOrientation::RunDiagonal,
                            reference(glm::cross(direction(start, upper), direction(start, end))));
                    }
                // Upper interfaces retain their attachment geometry. Connecting
                // them does not create another braced panel.
                for (std::size_t lane = 0; lane < rules.lanes.size(); ++lane)
                    addMember(before.trackInterface[lane], after.trackInterface[lane],
                        SupportMemberRole::LongitudinalTie, SupportMemberOrientation::RunLongitudinal,
                        reference(direction(before.levels.back()[lane], before.trackInterface[lane])));
            }
        }

        // Append after all inner framing so selecting outer lines preserves
        // the core's nodes, members, foundations and local tie junctions.
        // Separate primary chains also keep each mounting host unambiguous.
        for (const auto& [tower, choice] : outerSupportTowers)
        {
            const auto lateral = glm::normalize(across(tower.levels.front()));
            for (std::size_t lane = 0; lane < 2; ++lane)
            {
                if ((lane == 0 && choice.sides == HybridOuterSupportSides::Right)
                    || (lane == 1 && choice.sides == HybridOuterSupportSides::Left))
                    continue;
                const auto innerBase = tower.levels.front()[lane];
                const auto innerTop = tower.levels.back()[lane];
                const auto outward = (lane == 0 ? -lateral : lateral);
                const auto base = nodePositions[innerBase] + choice.foundationOutset * outward;
                const auto top = nodePositions[innerTop] + choice.topOutset * outward;
                auto previousOuter = addNode(base, std::nullopt, true);
                std::vector<SupportElementId> connections;
                if (!tower.lowerLedger.empty()) connections.push_back(tower.lowerLedger[lane]);
                for (std::size_t level = 1; level < tower.levels.size(); ++level)
                    connections.push_back(tower.levels[level][lane]);
                for (const auto inner : connections)
                {
                    // A straight independently founded line intersects only
                    // existing transverse ledger elevations, including the
                    // local shoulder. Staggered run ties do not add more rows.
                    const double fraction = (nodePositions[inner].z - base.z) / (top.z - base.z);
                    const auto outer = addNode(base + fraction * (top - base), std::nullopt, false);
                    addMember(previousOuter, outer, SupportMemberRole::PrimaryPost,
                        SupportMemberOrientation::BentPost, reference(lateral));
                    addMember(lane == 0 ? outer : inner, lane == 0 ? inner : outer,
                        SupportMemberRole::LedgerCap, SupportMemberOrientation::BentTransverse,
                        reference(direction(innerBase, innerTop)));
                    previousOuter = outer;
                }
            }
        }

        if (hybridBent)
        {
            // The reference establishes face-mounted caps, not terminal seats.
            // Ledgers retain +Z; braces carry their selected directed face.
            // Run members use the outer lane face independently at each end.
            for (auto& member : structure.members)
            {
                if (member.role == SupportMemberRole::PrimaryPost) continue;
                SupportMemberMounting mounting;
                if (member.orientation == SupportMemberOrientation::BentTransverse)
                {
                    mounting.coverage = SupportMemberEndCoverage::OutsideSupport;
                    // A modest procedural visual overhang, not a joint detail.
                    mounting.overhang = 0.15 * recipe.memberSize;
                }
                else if (member.orientation == SupportMemberOrientation::BentDiagonal)
                {
                    mounting.face = transverseBraceFaces.at(member.id);
                    mounting.layer = SupportMemberMountingLayer::OutsideLedger;
                    mounting.separation = 0.02 * recipe.memberSize;
                }
                else if (connectedTowers && member.orientation == SupportMemberOrientation::RunDiagonal)
                {
                    mounting.layer = SupportMemberMountingLayer::OutsideLedger;
                    mounting.separation = 0.02 * recipe.memberSize;
                }
                const bool runMember = member.orientation == SupportMemberOrientation::RunLongitudinal
                    || member.orientation == SupportMemberOrientation::RunDiagonal;
                SupportMemberEndConnection start;
                SupportMemberEndConnection end;
                start.mounting = mounting;
                end.mounting = mounting;
                if (runMember)
                {
                    start.mounting->face = postSideFaces.at(member.startNodeId);
                    end.mounting->face = postSideFaces.at(member.endNodeId);
                }
                member.startConnection = start;
                member.endConnection = end;
            }
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
