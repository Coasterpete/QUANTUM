#include <quantum/coaster/SupportSolidGeometry.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <ranges>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <stdexcept>
#include <unordered_map>

namespace quantum::coaster
{
    namespace
    {
        [[nodiscard]] bool isFinite(const glm::dvec3& value) noexcept
        {
            return std::isfinite(value.x) && std::isfinite(value.y)
                && std::isfinite(value.z);
        }

        [[nodiscard]] glm::vec3 toFloat(const glm::dvec3& value) noexcept
        {
            return {static_cast<float>(value.x), static_cast<float>(value.y),
                static_cast<float>(value.z)};
        }

        // Resolved node positions keyed by SupportElementId. Ids are sparse
        // and structure-local, so a lookup map keeps the member loop linear
        // in member count instead of quadratic.
        using NodeLookup = std::unordered_map<SupportElementId, glm::dvec3>;
        // Nodes carrying a Foundation anchor, used to size that node's pad
        // from the members it actually carries.
        using FoundationLookup =
            std::unordered_map<SupportElementId, double>;

        [[nodiscard]] NodeLookup buildNodeLookup(
            const SupportStructure& structure)
        {
            NodeLookup lookup;
            lookup.reserve(structure.nodes.size() * 2);
            for (const SupportNode& node : structure.nodes)
            {
                if (!isFinite(node.position))
                {
                    throw std::invalid_argument(
                        "Support node positions must be finite.");
                }
                lookup.emplace(node.id, node.position);
            }
            return lookup;
        }

        [[nodiscard]] const glm::dvec3& requireNode(
            const NodeLookup& lookup,
            const SupportElementId nodeId)
        {
            const auto found = lookup.find(nodeId);
            if (found == lookup.end())
            {
                throw std::logic_error(
                    "Validated support member endpoints were not found.");
            }
            return found->second;
        }

        // A rectangular cross-section takes outerDimensions.x as its width and
        // .y as its depth. A circular one is validated to have equal
        // dimensions, so the first component is the diameter.
        [[nodiscard]] glm::dvec2 crossSection(
            const SupportMemberProfile& profile) noexcept
        {
            if (profile.shape == SupportMemberProfileShape::Circular)
            {
                return {profile.outerDimensions.x, profile.outerDimensions.x};
            }
            return profile.outerDimensions;
        }

        // Support function of the cross-section in the requested direction.
        // A brace's broad face can be Y while a ledger's is Z; neither gets a
        // hard-coded thickness axis. Circular sections use the ellipse radius.
        [[nodiscard]] double sectionHalfExtent(const SupportMemberProfile& profile,
            const SupportMemberFrame& frame, const glm::dvec3& normal)
        {
            const auto section = crossSection(profile);
            const double y = section.x * glm::dot(normal, frame.axisY);
            const double z = section.y * glm::dot(normal, frame.axisZ);
            return 0.5 * (profile.shape == SupportMemberProfileShape::Circular
                ? std::hypot(y, z) : std::abs(y) + std::abs(z));
        }

        [[nodiscard]] glm::dvec3 mountingNormal(const SupportMemberMountingFace face,
            const SupportMemberFrame& host)
        {
            switch (face)
            {
            case SupportMemberMountingFace::PositiveY: return host.axisY;
            case SupportMemberMountingFace::NegativeY: return -host.axisY;
            case SupportMemberMountingFace::PositiveZ: return host.axisZ;
            case SupportMemberMountingFace::NegativeZ: return -host.axisZ;
            case SupportMemberMountingFace::PositiveX: return host.axisX;
            case SupportMemberMountingFace::NegativeX: return -host.axisX;
            }
            throw std::invalid_argument("Unknown support mounting face.");
        }

        // The unit meshes are unit-cube and unit-diameter, so the basis
        // columns carry the member's length, width, and depth directly.
        [[nodiscard]] glm::mat4 memberTransform(
            const SupportMemberFrame& frame,
            const glm::dvec2& section) noexcept
        {
            glm::mat4 transform{1.0F};
            transform[0] = glm::vec4(
                toFloat(frame.axisX) * static_cast<float>(frame.length),
                0.0F);
            transform[1] = glm::vec4(
                toFloat(frame.axisY) * static_cast<float>(section.x), 0.0F);
            transform[2] = glm::vec4(
                toFloat(frame.axisZ) * static_cast<float>(section.y), 0.0F);
            transform[3] = glm::vec4(toFloat(frame.origin), 1.0F);
            return transform;
        }
    }

    std::size_t SupportSolidPresentation::memberCount() const noexcept
    {
        std::size_t total = 0;
        for (const SupportSolidBatch& batch : batches)
        {
            total += batch.instances.size();
        }
        return total;
    }

    std::size_t SupportSolidPresentation::drawCallCount() const noexcept
    {
        std::size_t calls = 0;
        for (const SupportSolidBatch& batch : batches)
        {
            if (!batch.instances.empty())
            {
                ++calls;
            }
        }
        return calls;
    }

    bool SupportSolidPresentation::empty() const noexcept
    {
        return memberCount() == 0 && foundations.empty();
    }

    SupportMemberFrame resolveSupportMemberFrame(
        const glm::dvec3& start,
        const glm::dvec3& end,
        const SupportMemberOrientation orientation,
        const SupportMemberOrientationReference& orientationReference)
    {
        if (!isFinite(start) || !isFinite(end))
        {
            throw std::invalid_argument(
                "Support member endpoints must be finite.");
        }
        validateSupportMemberOrientation(orientation);
        if (orientationReference.has_value()
            && !isFinite(*orientationReference))
        {
            throw std::invalid_argument(
                "Support member orientation reference must be finite.");
        }

        SupportMemberFrame frame;
        const glm::dvec3 delta = end - start;
        frame.length = glm::length(delta);
        frame.origin = (start + end) * 0.5;
        if (frame.length <= minimumSupportMemberLength)
        {
            return frame;
        }

        frame.axisX = delta / frame.length;

        // Ordered reference preferences. Generic is the historical angle
        // branch, preserved exactly for legacy and manual members. Authored
        // planes name their reference directly: posts take the vertical
        // branch whatever their batter, everything else takes world +Z.
        const bool genericVertical =
            orientation == SupportMemberOrientation::Generic
            && std::abs(frame.axisX.z) > supportMemberVerticalTolerance;
        const bool preferVertical = orientation
                == SupportMemberOrientation::BentPost
            || genericVertical;
        glm::dvec3 reference = preferVertical
            ? glm::dvec3(1.0, 0.0, 0.0)
            : glm::dvec3(0.0, 0.0, 1.0);

        // Gram-Schmidt against the exact longitudinal axis. The reference is
        // recomputed rather than reused so a fallback that happens to be
        // parallel still yields a valid basis.
        auto project = [&](const glm::dvec3& candidate) -> bool
        {
            frame.axisY = candidate
                - frame.axisX * glm::dot(frame.axisX, candidate);
            const double perpendicularLength = glm::length(frame.axisY);
            if (perpendicularLength <= minimumSupportMemberLength)
            {
                return false;
            }
            frame.axisY /= perpendicularLength;
            return true;
        };

        // An authored reference wins outright: it is the directed bent frame
        // the member was generated with, so banked bents resolve in their
        // actual structural frame. A reference numerically parallel to the
        // member axis (stale after an edit) falls back deterministically
        // instead of corrupting the presentation.
        bool usedAuthoredReference = false;
        if (orientationReference.has_value()
            && project(*orientationReference))
        {
            usedAuthoredReference = true;
        }
        else if (!project(reference))
        {
            // The member runs parallel to its orientation reference (a
            // misauthored plane, or a stale authored vector after an edit).
            // Mirror once; bare Generic keeps its historical failure instead
            // so old documents resolve exactly as before.
            if (orientation == SupportMemberOrientation::Generic
                && !orientationReference.has_value())
            {
                throw std::invalid_argument(
                    "Support member orientation is degenerate.");
            }
            reference = preferVertical ? glm::dvec3(0.0, 0.0, 1.0)
                                       : glm::dvec3(1.0, 0.0, 0.0);
            if (!project(reference))
            {
                throw std::invalid_argument(
                    "Support member orientation is degenerate.");
            }
        }
        frame.usedVerticalFallback = !usedAuthoredReference
            && reference.x > 0.5;
        // Right-handed completion, so the local basis always maps +Y x +Z to
        // +X and triangle winding is preserved for every member.
        frame.axisZ = glm::cross(frame.axisX, frame.axisY);
        return frame;
    }

    std::vector<SupportMemberPlacement> resolveSupportMemberPlacements(
        const SupportStructure& structure)
    {
        if (std::ranges::any_of(structure.members, &SupportMember::useStyleProfile))
        {
            auto effective = structure;
            for (auto& member : effective.members)
            {
                member.profile = resolveSupportProfile(structure, member);
                member.useStyleProfile = false;
            }
            return resolveSupportMemberPlacements(effective);
        }
        const NodeLookup nodes = buildNodeLookup(structure);
        std::vector<SupportMemberPlacement> placements;
        placements.reserve(structure.members.size());
        bool hasMounting = false;
        for (const auto& member : structure.members)
        {
            validateSupportMemberProfile(member.profile);
            const auto& start = requireNode(nodes, member.startNodeId);
            const auto& end = requireNode(nodes, member.endNodeId);
            placements.push_back({member.id, start, end,
                resolveSupportMemberFrame(start, end, member.orientation, member.orientationReference)});
            for (const auto* connection : {&member.startConnection, &member.endConnection})
                if (*connection && (*connection)->mounting) hasMounting = true;
        }
        // The historical path is returned without arithmetic on endpoints.
        if (!hasMounting) return placements;

        using IncidentLookup = std::unordered_map<SupportElementId, std::vector<std::size_t>>;
        IncidentLookup posts;
        IncidentLookup ledgers;
        for (std::size_t i = 0; i < structure.members.size(); ++i)
        {
            const auto& member = structure.members[i];
            for (const auto node : {member.startNodeId, member.endNodeId})
            {
                if (member.role == SupportMemberRole::PrimaryPost) posts[node].push_back(i);
                // Run-side braces must also clear incident horizontal ties on
                // their selected post face, using each tie's resolved envelope.
                if (member.role == SupportMemberRole::LedgerCap
                    || member.role == SupportMemberRole::LongitudinalTie) ledgers[node].push_back(i);
            }
        }
        // Orient each post chain away from its foundation using connectivity,
        // not world height or the mounted member's endpoint order. Unanchored
        // manual chains use their lowest node ID as a deterministic root.
        std::unordered_map<SupportElementId, std::size_t> depth;
        const auto walk = [&](const SupportElementId root)
        {
            if (depth.contains(root) || !posts.contains(root)) return;
            std::vector<SupportElementId> queue{root};
            depth.emplace(root, 0);
            for (std::size_t head = 0; head < queue.size(); ++head)
            {
                const auto node = queue[head];
                for (const auto index : posts.at(node))
                {
                    const auto& post = structure.members[index];
                    const auto next = post.startNodeId == node ? post.endNodeId : post.startNodeId;
                    if (depth.emplace(next, depth.at(node) + 1).second) queue.push_back(next);
                }
            }
        };
        std::vector<SupportElementId> orderedNodes;
        for (const auto& node : structure.nodes) orderedNodes.push_back(node.id);
        std::sort(orderedNodes.begin(), orderedNodes.end());
        for (const auto& node : structure.nodes) if (node.foundation) walk(node.id);
        for (const auto node : orderedNodes) walk(node);

        std::vector<SupportMemberFrame> postFrames(placements.size());
        for (std::size_t i = 0; i < structure.members.size(); ++i)
        {
            const auto& post = structure.members[i];
            if (post.role != SupportMemberRole::PrimaryPost) continue;
            if ((post.startConnection && post.startConnection->mounting)
                || (post.endConnection && post.endConnection->mounting))
                throw std::invalid_argument("Mounting hosts must remain on their logical post chains.");
            const bool forward = depth.at(post.startNodeId) < depth.at(post.endNodeId);
            postFrames[i] = resolveSupportMemberFrame(
                forward ? placements[i].start : placements[i].end,
                forward ? placements[i].end : placements[i].start,
                post.orientation, post.orientationReference);
        }
        const auto hostAt = [&](const SupportElementId node) -> std::size_t
        {
            const auto found = posts.find(node);
            if (found == posts.end())
                throw std::invalid_argument("Mounted support endpoint has no incident primary post.");
            std::optional<std::size_t> lower;
            for (const auto index : found->second)
            {
                const auto& post = structure.members[index];
                const auto other = post.startNodeId == node ? post.endNodeId : post.startNodeId;
                if (depth.at(other) < depth.at(node))
                {
                    if (lower) throw std::invalid_argument("Mounted endpoint has ambiguous supporting post branches.");
                    lower = index;
                }
            }
            if (lower) return *lower;
            if (found->second.size() != 1)
                throw std::invalid_argument("Mounted endpoint has ambiguous supporting post branches.");
            return found->second.front();
        };
        std::vector<std::array<glm::dvec3, 2>> mountedEndpoints(placements.size());
        const auto resolve = [&](const std::size_t index)
        {
            const auto& member = structure.members[index];
            auto& placement = placements[index];
            const std::array<SupportElementId, 2> ids{member.startNodeId, member.endNodeId};
            const std::array<const std::optional<SupportMemberEndConnection>*, 2> connections{
                &member.startConnection, &member.endConnection};
            std::array<const SupportMemberMounting*, 2> mounting{};
            std::array<glm::dvec3, 2> normals{};
            std::array<double, 2> faceDistance{};
            std::array<std::size_t, 2> hosts{};
            const std::array<glm::dvec3, 2> logical{
                requireNode(nodes, ids[0]), requireNode(nodes, ids[1])};
            for (std::size_t end = 0; end < 2; ++end)
            {
                if (!*connections[end] || !(*connections[end])->mounting) continue;
                mounting[end] = &*(*connections[end])->mounting;
                const auto& intent = *mounting[end];
                validateSupportMemberMounting(intent);
                hosts[end] = hostAt(ids[end]);
                const auto& host = postFrames[hosts[end]];
                if (host.length <= minimumSupportMemberLength)
                    throw std::invalid_argument("Mounting requires a nondegenerate supporting post.");
                normals[end] = mountingNormal(intent.face, host);
                if (intent.mode == SupportMemberMountingMode::TerminalSeat)
                {
                    const bool positive = intent.face == SupportMemberMountingFace::PositiveX;
                    const auto& post = structure.members[hosts[end]];
                    const auto other = post.startNodeId == ids[end] ? post.endNodeId : post.startNodeId;
                    if (posts.at(ids[end]).size() != 1
                        || positive != (depth.at(ids[end]) > depth.at(other)))
                        throw std::invalid_argument("A terminal seat must face out of a terminal post end.");
                }
                else
                    faceDistance[end] = sectionHalfExtent(structure.members[hosts[end]].profile,
                        host, normals[end]);
                if (intent.layer == SupportMemberMountingLayer::OutsideLedger)
                {
                    const auto row = ledgers.find(ids[end]);
                    if (row != ledgers.end()) for (const auto ledgerIndex : row->second)
                    {
                        if (ledgerIndex == index) continue;
                        const auto& ledger = structure.members[ledgerIndex];
                        const std::size_t ledgerEnd = ledger.startNodeId == ids[end] ? 0 : 1;
                        const auto& connection = ledgerEnd == 0 ? ledger.startConnection : ledger.endConnection;
                        if (!connection || !connection->mounting) continue;
                        const auto& ledgerIntent = *connection->mounting;
                        if (ledgerIntent.mode != SupportMemberMountingMode::Face
                            || ledgerIntent.face != intent.face) continue;
                        if (ledgerIntent.layer != SupportMemberMountingLayer::Direct)
                            throw std::invalid_argument("OutsideLedger requires directly mounted incident ledgers.");
                        const double outer = glm::dot(mountedEndpoints[ledgerIndex][ledgerEnd]
                            - logical[end], normals[end]) + sectionHalfExtent(ledger.profile,
                                placements[ledgerIndex].frame, normals[end]);
                        faceDistance[end] = std::max(faceDistance[end], outer);
                    }
                }
                faceDistance[end] += intent.separation;
            }
            auto frame = resolveSupportMemberFrame(logical[0], logical[1], member.orientation,
                member.orientationReference);
            std::array<glm::dvec3, 2> physical = logical;
            // Different host normals change the final member axis. Re-evaluate
            // its projected half-section against that axis until contact
            // distances agree, instead of using a stale logical-frame width.
            bool converged = false;
            for (int iteration = 0; iteration < 64; ++iteration)
            {
                auto next = logical;
                for (std::size_t end = 0; end < 2; ++end)
                    if (mounting[end]) next[end] += normals[end] * (faceDistance[end]
                        + sectionHalfExtent(member.profile, frame, normals[end]));
                const double error = std::max(glm::length(next[0] - physical[0]),
                    glm::length(next[1] - physical[1]));
                physical = next;
                frame = resolveSupportMemberFrame(physical[0], physical[1], member.orientation,
                    member.orientationReference);
                if (error <= 1e-11) { converged = true; break; }
            }
            if (!converged) throw std::invalid_argument("Support mounting contact resolution did not converge.");
            mountedEndpoints[index] = physical;
            for (std::size_t end = 0; end < 2; ++end)
            {
                if (!mounting[end]) continue;
                double extension = mounting[end]->overhang;
                if (mounting[end]->coverage == SupportMemberEndCoverage::OutsideSupport)
                {
                    const auto& host = postFrames[hosts[end]];
                    const auto section = crossSection(structure.members[hosts[end]].profile);
                    const double y = std::abs(glm::dot(frame.axisX, host.axisY));
                    const double z = std::abs(glm::dot(frame.axisX, host.axisZ));
                    // Cover the supporting side plane most aligned with the
                    // member axis. Division accounts for a skewed connection.
                    if (std::max(y, z) <= minimumSupportMemberLength)
                        throw std::invalid_argument("End coverage requires a member crossing a supporting side face.");
                    extension += y >= z ? 0.5 * section.x / y : 0.5 * section.y / z;
                }
                physical[end] += (end == 0 ? -frame.axisX : frame.axisX) * extension;
            }
            placement.start = physical[0];
            placement.end = physical[1];
            placement.frame = resolveSupportMemberFrame(physical[0], physical[1], member.orientation,
                member.orientationReference);
        };
        // Direct contacts first; the outer brace layer reads their resolved
        // ledger envelopes. This fixed dependency order needs no joint manager.
        for (const auto layer : {SupportMemberMountingLayer::Direct, SupportMemberMountingLayer::OutsideLedger})
            for (std::size_t i = 0; i < structure.members.size(); ++i)
            {
                const auto& member = structure.members[i];
                const bool outer = (member.startConnection && member.startConnection->mounting
                    && member.startConnection->mounting->layer == SupportMemberMountingLayer::OutsideLedger)
                    || (member.endConnection && member.endConnection->mounting
                    && member.endConnection->mounting->layer == SupportMemberMountingLayer::OutsideLedger);
                if (outer == (layer == SupportMemberMountingLayer::OutsideLedger)) resolve(i);
            }
        return placements;
    }

    SupportSolidPresentation buildSupportSolidPresentation(
        const SupportStructure& structure, const double metersPerCoordinateUnit)
    {
        if (!std::isfinite(metersPerCoordinateUnit) || metersPerCoordinateUnit <= 0)
            throw std::invalid_argument("Support presentation requires a positive coordinate scale.");
        SupportSolidPresentation presentation;
        presentation.metersPerCoordinateUnit = metersPerCoordinateUnit;
        presentation.structureId = structure.id;
        presentation.appearance =
            structure.appearance.value_or(SupportAppearance{});
        presentation.foundationAppearance =
            structure.foundationAppearance.value_or(
                SupportFoundationAppearance{});
        validateSupportAppearance(presentation.appearance);
        validateSupportFoundationAppearance(
            presentation.foundationAppearance);

        const auto placements = resolveSupportMemberPlacements(structure);

        const auto append = [&](SupportMemberMeshKind mesh, const glm::mat4& transform,
            const SupportSurface& surface, const glm::vec4& section = glm::vec4{1, 0, 0, 0})
        {
            auto batch = std::ranges::find_if(presentation.batches, [&](const auto& b)
                { return b.mesh == mesh && b.surface == surface; });
            if (batch == presentation.batches.end())
            {
                presentation.batches.push_back({mesh, {}, surface});
                batch = presentation.batches.end() - 1;
            }
            batch->instances.push_back({transform, section});
        };

        // The largest cross-section dimension meeting each foundation node.
        // Indexing these IDs avoids searching every node for every member.
        // This is a presentation proportion, not structural footing design.
        FoundationLookup foundationSections;
        foundationSections.reserve(structure.nodes.size());
        for (const SupportNode& node : structure.nodes)
        {
            if (node.foundation)
            {
                foundationSections.emplace(node.id, 0.0);
            }
        }

        for (std::size_t index = 0; index < structure.members.size(); ++index)
        {
            const SupportMember& member = structure.members[index];
            validateSupportMemberProfile(member.profile);
            validateSupportMemberOrientation(member.orientation);
            validateSupportMemberOrientationReference(
                member.orientationReference);
            const SupportMemberFrame& frame = placements[index].frame;
            if (frame.length <= minimumSupportMemberLength)
            {
                // The generator already rejects these, so reaching one here
                // means externally written or hand-edited data. Skipping keeps
                // a sub-pixel or NaN basis out of the instance buffer without
                // failing the whole presentation.
                continue;
            }

            const auto profile = resolveSupportProfile(structure, member);
            const glm::dvec2 section = crossSection(profile);
            const auto surface = resolveSupportSurface(structure, member, metersPerCoordinateUnit);
            validateSupportSurface(surface);
            const auto part = [&](const glm::dvec2& dimensions, const glm::dvec2& offset)
            {
                auto f = frame;
                f.origin += f.axisY * offset.x + f.axisZ * offset.y;
                append(SupportMemberMeshKind::Rectangular, memberTransform(f, dimensions), surface);
            };
            const double t = profile.wallThickness;
            if (profile.shape == SupportMemberProfileShape::Hss
                || (profile.shape == SupportMemberProfileShape::Rectangular && t > 0))
            {
                part({t, section.y}, {-(section.x - t) * .5, 0});
                part({t, section.y}, {(section.x - t) * .5, 0});
                part({section.x - 2*t, t}, {0, -(section.y - t) * .5});
                part({section.x - 2*t, t}, {0, (section.y - t) * .5});
            }
            else if (profile.shape == SupportMemberProfileShape::ISection)
            {
                part({section.x, t}, {0, -(section.y - t) * .5});
                part({section.x, t}, {0, (section.y - t) * .5});
                part({t, section.y - 2*t}, {0, 0});
            }
            else if (profile.shape == SupportMemberProfileShape::Circular)
            {
                append(t > 0 ? SupportMemberMeshKind::CircularTube : SupportMemberMeshKind::Circular,
                    memberTransform(frame, section), surface,
                    {static_cast<float>(1 - 2*t/section.x), 1, 0, 0});
            }
            else part(section, {0, 0});

            // Connections remain identified by member + directed end. Their
            // display frame follows physical mounting, including authored GLB
            // placement offsets; no new graph objects or interlock state.
            for (int end = 0; end < 2; ++end)
            {
                const auto& connection = end == 0 ? member.startConnection : member.endConnection;
                if (!connection || connection->treatment == SupportMemberEndTreatment::MiteredCut) continue;
                auto f = frame;
                f.origin = end == 0 ? placements[index].start : placements[index].end;
                if (end == 0) { f.axisX = -f.axisX; f.axisZ = -f.axisZ; }
                glm::dmat4 placement{1.0};
                if (connection->localPlacement)
                {
                    const auto& p = *connection->localPlacement;
                    placement = glm::translate(glm::dmat4{1}, p.position) * glm::mat4_cast(p.orientation)
                        * glm::scale(glm::dmat4{1}, p.scale);
                }
                const auto steel = defaultSupportSurface(SupportMaterialKind::GalvanizedSteel);
                const double size = std::max(section.x, section.y);
                const auto box = [&](glm::dvec3 offset, glm::dvec3 dimensions, bool concrete = false)
                {
                    glm::dmat4 basis{1};
                    basis[0] = glm::dvec4{f.axisX, 0}; basis[1] = glm::dvec4{f.axisY, 0};
                    basis[2] = glm::dvec4{f.axisZ, 0}; basis[3] = glm::dvec4{f.origin, 1};
                    auto transform = basis * placement * glm::translate(glm::dmat4{1}, offset)
                        * glm::scale(glm::dmat4{1}, dimensions);
                    append(SupportMemberMeshKind::Rectangular, glm::mat4{transform},
                        concrete ? defaultSupportSurface(SupportMaterialKind::Concrete) : steel, {1, 0, 0, 1});
                };
                using T = SupportMemberEndTreatment;
                switch (connection->treatment)
                {
                case T::MiteredCut: break;
                case T::EndCap: box({0,0,0}, {.06*size, section.x*1.04, section.y*1.04}); break;
                case T::Plate: box({0,0,0}, {.12*size, section.x*1.6, section.y*1.6}); break;
                case T::Flange:
                case T::Base:
                    box({0,0,0}, {.15*size, section.x*2, section.y*2});
                    box({-.2*size,0,0}, {.35*size, section.x*1.12, section.y*1.12}); break;
                case T::Splice:
                    for (double side : {-1.0,1.0}) box({-.4*size,side*.55*section.x,0},
                        {size,.1*size,section.y*1.1}); break;
                case T::Saddle:
                case T::Clamp:
                    box({0,0,0}, {.18*size, section.x*1.8, section.y*1.4});
                    for (double side : {-1.0,1.0}) box({.3*size,side*.8*section.x,0},
                        {.6*size,.15*size,section.y*1.4});
                    if (connection->treatment == T::Clamp)
                        box({.6*size,0,0}, {.12*size, section.x*1.8, section.y*1.4});
                    break;
                case T::Footing: box({0,0,0}, {.2*size,section.x*2,section.y*2}, true); break;
                }
            }

            const double largest = std::max(section.x, section.y);
            for (const SupportElementId nodeId :
                {member.startNodeId, member.endNodeId})
            {
                const auto found = foundationSections.find(nodeId);
                if (found != foundationSections.end())
                {
                    found->second = std::max(found->second, largest);
                }
            }
        }

        // Keep historical profile ordering while also grouping by resolved material.
        std::stable_sort(presentation.batches.begin(), presentation.batches.end(),
            [](const auto& a, const auto& b) { return a.mesh < b.mesh; });

        for (const SupportNode& node : structure.nodes)
        {
            if (!node.foundation.has_value())
            {
                continue;
            }
            SupportFoundationPad pad;
            pad.position = node.position;
            const auto found = foundationSections.find(node.id);
            const double carried =
                found == foundationSections.end() ? 0.0 : found->second;
            const double footprint = carried > 0.0
                ? carried * supportFoundationPadFootprintScale
                : supportFoundationPadFootprintScale;
            pad.padDimensions = {
                presentation.foundationAppearance.padDimensions.x > 0.0
                    ? presentation.foundationAppearance.padDimensions.x
                    : footprint,
                presentation.foundationAppearance.padDimensions.y > 0.0
                    ? presentation.foundationAppearance.padDimensions.y
                    : footprint};
            // Thickness comes from the post's own cross-section, not from the
            // pad footprint. Scaling the footprint instead would make a footing
            // as thick as it is wide, which reads as a square pillar stub.
            const double section = carried;
            pad.padDepth =
                presentation.foundationAppearance.padDepth > 0.0
                ? presentation.foundationAppearance.padDepth
                : (section > 0.0 ? section : 1.0)
                    * supportFoundationPadDepthScale;
            pad.kind = node.foundation->kind;
            const auto dimensions = node.foundation->dimensions;
            if (dimensions.x > 0) pad.padDimensions.x = dimensions.x;
            if (dimensions.y > 0) pad.padDimensions.y = dimensions.y;
            if (dimensions.z > 0) pad.padDepth = dimensions.z;
            if (pad.kind != SupportFoundationKind::Footing && dimensions.z == 0)
                pad.padDepth *= pad.kind == SupportFoundationKind::Pier ? 6.0 : 2.0;
            presentation.foundations.push_back(pad);
        }

        return presentation;
    }

    std::vector<SupportSolidPresentation> buildSupportSolidPresentation(
        const SupportCollection& collection, const double metersPerCoordinateUnit)
    {
        std::vector<SupportSolidPresentation> presentations;
        presentations.reserve(collection.structures.size());
        for (const SupportStructure& structure : collection.structures)
        {
            presentations.push_back(buildSupportSolidPresentation(structure, metersPerCoordinateUnit));
        }
        return presentations;
    }
    glm::dvec2 supportSurfaceUv(const glm::dvec3& p, const glm::dvec3& n,
        const glm::dvec3& extent, const glm::dvec2& repeat, bool circular, double seamFraction)
    {
        if (!isFinite(p) || !isFinite(n) || !isFinite(extent)
            || extent.x <= 0 || extent.y <= 0 || extent.z <= 0
            || !std::isfinite(repeat.x) || !std::isfinite(repeat.y) || repeat.x <= 0 || repeat.y <= 0
            || !std::isfinite(seamFraction)) throw std::invalid_argument("Invalid support UV input.");
        if (std::abs(n.x) >= .5) return {p.y / repeat.x, p.z * (n.x > 0 ? 1 : -1) / repeat.y};
        if (circular) return {p.x / repeat.x, seamFraction * std::numbers::pi * extent.y * (n.y*p.y + n.z*p.z > 0 ? -1 : 1) / repeat.y};
        const double across = std::abs(n.y) > .5 ? -std::copysign(1.0, n.y)*p.z : std::copysign(1.0, n.z)*p.y;
        return {p.x / repeat.x, across / repeat.y};
    }

}
