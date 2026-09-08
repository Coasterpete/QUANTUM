#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/Supports.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace
{
    using namespace quantum::coaster;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    template<typename Function>
    void requireInvalid(Function&& function, const std::string_view message)
    {
        try
        {
            std::forward<Function>(function)();
        }
        catch (const std::invalid_argument&)
        {
            return;
        }
        throw std::runtime_error(std::string(message));
    }

    [[nodiscard]] SupportMemberProfile roundTube()
    {
        return {SupportMemberProfileShape::Circular, {0.3, 0.3}, 0.025};
    }

    [[nodiscard]] SupportMemberProfile timber()
    {
        return {SupportMemberProfileShape::Rectangular, {0.2, 0.3}, 0.0};
    }

    SupportStructure& addStructure(
        SupportCollection& collection,
        std::string name)
    {
        const SupportStructureId id = allocateSupportStructureId(collection);
        collection.structures.push_back({id, std::move(name)});
        return collection.structures.back();
    }

    SupportElementId addNode(
        SupportStructure& structure,
        const glm::dvec3 position)
    {
        const SupportElementId id = allocateSupportElementId(structure);
        structure.nodes.push_back({id, position});
        return id;
    }

    void addMember(
        SupportStructure& structure,
        const SupportElementId start,
        const SupportElementId end,
        const SupportMemberProfile& profile)
    {
        structure.members.push_back({
            allocateSupportElementId(structure), start, end, profile});
    }

    void emptyCollectionIsValid()
    {
        const SupportCollection collection;
        validateSupportCollection(collection);
        require(collection.empty(), "default support collection must be empty");

        const AuthoredTrack track;
        validateSupportCollection(track.supports());
        require(track.supports().empty(),
            "default AuthoredTrack must own an empty valid support collection");
    }

    void allocationIsMonotonicAndDoesNotReuseIds()
    {
        SupportCollection collection;
        const auto firstStructure = allocateSupportStructureId(collection);
        collection.structures.push_back({firstStructure, "First"});
        const auto secondStructure = allocateSupportStructureId(collection);
        collection.structures.push_back({secondStructure, "Second"});
        collection.structures.erase(collection.structures.begin());
        const auto thirdStructure = allocateSupportStructureId(collection);
        require(firstStructure == 1 && secondStructure == 2
                && thirdStructure == 3,
            "structure IDs must increase without gap reuse");

        SupportStructure& structure = collection.structures.front();
        const auto firstElement = addNode(structure, {0.0, 0.0, 0.0});
        const auto secondElement = addNode(structure, {0.0, 0.0, 1.0});
        structure.nodes.erase(structure.nodes.begin());
        const auto thirdElement = addNode(structure, {0.0, 0.0, 2.0});
        require(firstElement == 1 && secondElement == 2 && thirdElement == 3,
            "element IDs must increase without gap reuse");
    }

    void invalidIdsAndReferencesAreRejected()
    {
        SupportCollection collection;
        SupportStructure& structure = addStructure(collection, "Frame");
        const auto first = addNode(structure, {0.0, 0.0, 0.0});
        const auto second = addNode(structure, {0.0, 0.0, 1.0});
        addMember(structure, first, second, roundTube());
        validateSupportCollection(collection);

        auto zeroStructure = collection;
        zeroStructure.structures.front().id = 0;
        requireInvalid([&] { validateSupportCollection(zeroStructure); },
            "zero structure ID must be rejected");

        auto duplicateStructure = collection;
        duplicateStructure.structures.push_back(
            duplicateStructure.structures.front());
        requireInvalid([&] { validateSupportCollection(duplicateStructure); },
            "duplicate structure ID must be rejected");

        auto zeroElement = collection;
        zeroElement.structures.front().nodes.front().id = 0;
        requireInvalid([&] { validateSupportCollection(zeroElement); },
            "zero element ID must be rejected");

        auto duplicateElement = collection;
        duplicateElement.structures.front().members.front().id = first;
        requireInvalid([&] { validateSupportCollection(duplicateElement); },
            "node/member IDs share one unique element namespace");

        auto dangling = collection;
        dangling.structures.front().members.front().endNodeId = 999;
        requireInvalid([&] { validateSupportCollection(dangling); },
            "dangling member endpoint must be rejected");

        auto selfMember = collection;
        selfMember.structures.front().members.front().endNodeId = first;
        requireInvalid([&] { validateSupportCollection(selfMember); },
            "self-connected member must be rejected");
    }

    void invalidGeometryAndCountersAreRejected()
    {
        SupportCollection collection;
        SupportStructure& structure = addStructure(collection, "Column");
        const auto bottom = addNode(structure, {0.0, 0.0, 0.0});
        const auto top = addNode(structure, {0.0, 0.0, 5.0});
        addMember(structure, bottom, top, roundTube());

        auto nonfinite = collection;
        nonfinite.structures.front().nodes.front().position.x =
            std::numeric_limits<double>::infinity();
        requireInvalid([&] { validateSupportCollection(nonfinite); },
            "non-finite node position must be rejected");

        auto invalidDimensions = collection;
        invalidDimensions.structures.front().members.front()
            .profile.outerDimensions.x = 0.0;
        requireInvalid([&] { validateSupportCollection(invalidDimensions); },
            "non-positive profile dimension must be rejected");

        auto invalidWall = collection;
        invalidWall.structures.front().members.front().profile.wallThickness =
            0.15;
        requireInvalid([&] { validateSupportCollection(invalidWall); },
            "wall consuming the profile must be rejected");

        auto invalidShape = collection;
        invalidShape.structures.front().members.front().profile.shape =
            static_cast<SupportMemberProfileShape>(255);
        requireInvalid([&] { validateSupportCollection(invalidShape); },
            "unsupported profile shape must be rejected");

        auto invalidElementCounter = collection;
        invalidElementCounter.structures.front().nextElementId =
            invalidElementCounter.structures.front().members.front().id;
        requireInvalid([&] { validateSupportCollection(invalidElementCounter); },
            "element counter capable of reuse must be rejected");

        auto invalidStructureCounter = collection;
        invalidStructureCounter.nextStructureId =
            invalidStructureCounter.structures.front().id;
        requireInvalid([&] { validateSupportCollection(invalidStructureCounter); },
            "structure counter capable of reuse must be rejected");
    }

    void steelGraphsUseTheCommonModel()
    {
        SupportCollection collection;
        SupportStructure& column = addStructure(collection, "Steel column");
        const auto base = addNode(column, {0.0, 0.0, 0.0});
        const auto top = addNode(column, {0.0, 0.0, 8.0});
        addMember(column, base, top, roundTube());

        SupportStructure& frame = addStructure(collection, "A/V frame");
        const auto left = addNode(frame, {-2.0, 0.0, 0.0});
        const auto right = addNode(frame, {2.0, 0.0, 0.0});
        const auto center = addNode(frame, {0.0, 0.0, 6.0});
        const auto upper = addNode(frame, {0.0, 0.0, 9.0});
        addMember(frame, left, center, roundTube());
        addMember(frame, right, center, roundTube());
        addMember(frame, center, upper, roundTube());
        validateSupportCollection(collection);
    }

    void woodenBentRunUsesTheCommonModel()
    {
        SupportCollection collection;
        SupportStructure& bent = addStructure(collection, "Wood bent");
        const auto leftBottom = addNode(bent, {-2.0, 0.0, 0.0});
        const auto rightBottom = addNode(bent, {2.0, 0.0, 0.0});
        const auto leftTop = addNode(bent, {-2.0, 0.0, 6.0});
        const auto rightTop = addNode(bent, {2.0, 0.0, 6.0});
        const auto leftMid = addNode(bent, {-2.0, 0.0, 3.0});
        const auto rightMid = addNode(bent, {2.0, 0.0, 3.0});
        const auto nextLeftBottom = addNode(bent, {-2.0, 4.0, 0.0});
        const auto nextRightBottom = addNode(bent, {2.0, 4.0, 0.0});
        const auto nextLeftTop = addNode(bent, {-2.0, 4.0, 6.0});
        const auto nextRightTop = addNode(bent, {2.0, 4.0, 6.0});
        addMember(bent, leftBottom, leftTop, timber());
        addMember(bent, rightBottom, rightTop, timber());
        addMember(bent, leftTop, rightTop, timber());
        addMember(bent, leftBottom, rightMid, timber());
        addMember(bent, rightBottom, leftMid, timber());
        addMember(bent, leftMid, rightMid, timber());
        addMember(bent, nextLeftBottom, nextLeftTop, timber());
        addMember(bent, nextRightBottom, nextRightTop, timber());
        addMember(bent, nextLeftTop, nextRightTop, timber());
        addMember(bent, nextLeftBottom, nextRightTop, timber());
        addMember(bent, nextRightBottom, nextLeftTop, timber());
        addMember(bent, leftTop, nextLeftTop, timber());
        addMember(bent, rightTop, nextRightTop, timber());
        validateSupportCollection(collection);
    }

    void nodePositionEditingUsesStableIds()
    {
        SupportCollection collection;
        SupportStructure& first = addStructure(collection, "First");
        const auto untouchedNode = addNode(first, {1.0, 2.0, 3.0});
        SupportStructure& second = addStructure(collection, "Second");
        const auto targetNode = addNode(second, {4.0, 5.0, 6.0});

        AuthoredTrack track;
        track.setSupports(collection);
        track.setSupportNodePosition(second.id, targetNode, {7.0, 8.0, 9.0});

        require(track.supports().structures[0].nodes[0].id == untouchedNode
                && track.supports().structures[0].nodes[0].position
                    == glm::dvec3{1.0, 2.0, 3.0},
            "node editing must preserve unrelated support data");
        require(track.supports().structures[1].nodes[0].id == targetNode
                && track.supports().structures[1].nodes[0].position
                    == glm::dvec3{7.0, 8.0, 9.0},
            "node editing must change only the stable target ID");

        requireInvalid([&] {
            track.setSupportNodePosition(999, targetNode, {0.0, 0.0, 0.0});
        }, "unknown structure ID must be rejected");
        requireInvalid([&] {
            track.setSupportNodePosition(second.id, 999, {0.0, 0.0, 0.0});
        }, "unknown node ID must be rejected");
        requireInvalid([&] {
            track.setSupportNodePosition(second.id, targetNode,
                {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
        }, "non-finite node position must be rejected");
        require(track.supports().structures[1].nodes[0].position
                == glm::dvec3{7.0, 8.0, 9.0},
            "rejected node editing must leave the document unchanged");
    }

    void mutationApiEnforcesGraphInvariants()
    {
        SupportCollection collection;
        const SupportStructureId first =
            createSupportStructure(collection);
        const SupportStructureId second =
            createSupportStructure(collection, "A-frame");
        require(first == 1 && second == 2,
            "structure IDs must be allocated in order");
        require(collection.structures[0].name == "Support 1"
                && collection.structures[1].name == "A-frame",
            "unnamed structures must receive a default generated name");
        validateSupportCollection(collection);

        SupportStructure& secondStructure =
            collection.structures[1];
        const SupportElementId left = createSupportNode(
            collection, second, {-2.0, 0.0, 0.0});
        const SupportElementId right = createSupportNode(
            collection, second, {2.0, 0.0, 0.0});
        const SupportElementId top = createSupportNode(
            collection, second, {0.0, 0.0, 6.0});
        require(left == 1 && right == 2 && top == 3,
            "element IDs must be allocated in order within a structure");
        require(secondStructure.nodes.size() == 3
                && secondStructure.members.empty(),
            "node creation must not fabricate members");

        const SupportElementId leftMember = createSupportMember(
            collection, second, left, top, {SupportMemberProfileShape::Circular, {0.3, 0.3}, 0.025});
        const SupportElementId rightMember = createSupportMember(
            collection, second, right, top, roundTube());
        require(leftMember == 4 && rightMember == 5,
            "member IDs must share the element ID stream");
        validateSupportCollection(collection);

        requireInvalid([&] {
            createSupportMember(collection, second, top, top, roundTube());
        }, "self-connected member must be rejected");
        requireInvalid([&] {
            createSupportMember(collection, second, left, top, roundTube());
        }, "duplicate unordered node pair must be rejected");
        requireInvalid([&] {
            createSupportMember(collection, second, top, left, roundTube());
        }, "reversed duplicate node pair must be rejected");
        requireInvalid([&] {
            createSupportMember(collection, second, left, 999, roundTube());
        }, "missing endpoint node must be rejected");
        requireInvalid([&] {
            createSupportMember(
                collection, first, left, top, roundTube());
        }, "cross-structure endpoints must be rejected");

        requireInvalid([&] {
            removeSupportNode(collection, second, left);
        }, "referenced node removal must be rejected");
        validateSupportCollection(collection);

        removeSupportMember(collection, second, leftMember);
        createSupportMember(collection, second, left, top, roundTube());
        require(secondStructure.members.size() == 2,
            "removing a member must allow the pair to be re-created");

        const SupportElementId loose = createSupportNode(
            collection, second, {0.0, 5.0, 5.0});
        removeSupportNode(collection, second, loose);
        requireInvalid([&] {
            removeSupportNode(collection, second, loose);
        }, "removing an already-removed node must be rejected");
        requireInvalid([&] {
            removeSupportStructure(collection, 999);
        }, "removing an unknown structure must be rejected");

        removeSupportStructure(collection, first);
        require(collection.structures.size() == 1
                && collection.structures.front().id == second,
            "structure removal must remove the whole graph");
        validateSupportCollection(collection);

        const SupportStructureId third =
            createSupportStructure(collection);
        require(third == 3,
            "deleted structures must not cause ID reuse");
    }

    void woodenBentAuthoredWithMutationApi()
    {
        SupportCollection collection;
        const SupportStructureId bentId =
            createSupportStructure(collection, "Wood bent");
        const auto at = [&](const glm::dvec3 position)
        {
            return createSupportNode(collection, bentId, position);
        };
        const auto connect = [&](const SupportElementId start,
            const SupportElementId end)
        {
            return createSupportMember(collection, bentId, start, end,
                timber());
        };

        const SupportElementId leftBottom = at({-2.0, 0.0, 0.0});
        const SupportElementId rightBottom = at({2.0, 0.0, 0.0});
        const SupportElementId leftTop = at({-2.0, 0.0, 6.0});
        const SupportElementId rightTop = at({2.0, 0.0, 6.0});
        const SupportElementId leftMid = at({-2.0, 0.0, 3.0});
        const SupportElementId rightMid = at({2.0, 0.0, 3.0});
        const SupportElementId nextLeftBottom = at({-2.0, 4.0, 0.0});
        const SupportElementId nextRightBottom = at({2.0, 4.0, 0.0});
        const SupportElementId nextLeftTop = at({-2.0, 4.0, 6.0});
        const SupportElementId nextRightTop = at({2.0, 4.0, 6.0});
        connect(leftBottom, leftTop);
        connect(rightBottom, rightTop);
        connect(leftTop, rightTop);
        connect(leftBottom, rightMid);
        connect(rightBottom, leftMid);
        connect(leftMid, rightMid);
        connect(nextLeftBottom, nextLeftTop);
        connect(nextRightBottom, nextRightTop);
        connect(nextLeftTop, nextRightTop);
        connect(nextLeftBottom, nextRightTop);
        connect(nextRightBottom, nextLeftTop);
        connect(leftTop, nextLeftTop);
        const SupportElementId lastMember =
            connect(rightTop, nextRightTop);
        validateSupportCollection(collection);

        require(collection.structures.size() == 1,
            "the proof must stay a single structure");
        const SupportStructure& bent = collection.structures.front();
        require(bent.name == "Wood bent" && bent.id == bentId,
            "structure identity must be preserved");
        require(bent.nodes.size() == 10,
            "the wood bent proof must contain exactly ten nodes");
        require(bent.members.size() == 13,
            "the wood bent proof must contain exactly thirteen members");
        require(std::any_of(
            bent.members.begin(), bent.members.end(),
            [lastMember](const SupportMember& member)
            {
                return member.id == lastMember
                    && member.profile.shape
                        == SupportMemberProfileShape::Rectangular;
            }),
            "the last member must carry the timber profile");
    }

    void steelAFrameAuthoredWithMutationApi()
    {
        SupportCollection collection;
        const SupportStructureId columnId =
            createSupportStructure(collection, "Steel column");
        const SupportElementId base = createSupportNode(
            collection, columnId, {0.0, 0.0, 0.0});
        const SupportElementId columnTop = createSupportNode(
            collection, columnId, {0.0, 0.0, 8.0});
        const SupportElementId columnMember = createSupportMember(
            collection, columnId, base, columnTop, roundTube());

        const SupportStructureId frameId =
            createSupportStructure(collection, "A/V frame");
        const SupportElementId left = createSupportNode(
            collection, frameId, {-2.0, 0.0, 0.0});
        const SupportElementId right = createSupportNode(
            collection, frameId, {2.0, 0.0, 0.0});
        const SupportElementId center = createSupportNode(
            collection, frameId, {0.0, 0.0, 6.0});
        const SupportElementId upper = createSupportNode(
            collection, frameId, {0.0, 0.0, 9.0});
        createSupportMember(collection, frameId, left, center, roundTube());
        createSupportMember(collection, frameId, right, center, roundTube());
        createSupportMember(collection, frameId, center, upper, roundTube());
        validateSupportCollection(collection);

        require(collection.structures.size() == 2,
            "the steel proof must contain two structures");
        require(collection.structures[0].members.size() == 1
                && collection.structures[0].members.front().id
                    == columnMember,
            "the steel column must retain its single member");
        const SupportStructure& frame = collection.structures[1];
        require(frame.nodes.size() == 4 && frame.members.size() == 3,
            "the A/V frame must retain four nodes and three members");
        requireInvalid([&] {
            createSupportMember(
                collection, frameId, base, center, roundTube());
        }, "a member crossing structures must be rejected");
    }
}

int main()
{
    try
    {
        emptyCollectionIsValid();
        allocationIsMonotonicAndDoesNotReuseIds();
        invalidIdsAndReferencesAreRejected();
        invalidGeometryAndCountersAreRejected();
        steelGraphsUseTheCommonModel();
        woodenBentRunUsesTheCommonModel();
        nodePositionEditingUsesStableIds();
        mutationApiEnforcesGraphInvariants();
        woodenBentAuthoredWithMutationApi();
        steelAFrameAuthoredWithMutationApi();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Support model test failure: " << exception.what()
                  << '\n';
        return 1;
    }
}
