#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/SupportSolidGeometry.hpp>
#include <quantum/coaster/Supports.hpp>
#include <quantum/coaster/WoodenSupportGenerator.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace
{
    using namespace quantum::coaster;
    using json = nlohmann::json;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    [[nodiscard]] AuthoredTrack elevatedTrack()
    {
        AuthoredTrack track = createNewDocument();
        AuthoredStartPose pose = track.startPose();
        pose.position.z = 20.0;
        track.setStartPose(pose);
        return track;
    }

    void roleProfilesAreDistinctRectangles()
    {
        constexpr double memberSize = 0.2;
        const SupportMemberProfile post =
            timberProfileForRole(SupportMemberRole::PrimaryPost, memberSize);
        const SupportMemberProfile ledger =
            timberProfileForRole(SupportMemberRole::LedgerCap, memberSize);
        const SupportMemberProfile tie = timberProfileForRole(
            SupportMemberRole::LongitudinalTie, memberSize);
        const SupportMemberProfile brace =
            timberProfileForRole(SupportMemberRole::Brace, memberSize);
        const SupportMemberProfile trackSupport = timberProfileForRole(
            SupportMemberRole::TrackSupport, memberSize);

        for (const SupportMemberProfile* profile :
            {&post, &ledger, &tie, &brace, &trackSupport})
        {
            require(
                profile->shape == SupportMemberProfileShape::Rectangular,
                "every timber role must stay a rectangular timber section");
            require(
                profile->wallThickness == 0.0,
                "timber role profiles must be solid sections");
            validateSupportMemberProfile(*profile);
        }

        // Posts keep the nominal square so footing pads derived from the
        // carried section keep their M1/M2A footprint.
        require(
            post.outerDimensions.x == memberSize
                && post.outerDimensions.y == memberSize,
            "primary posts must keep the nominal square section");
        // Every other role is a distinct non-square rectangle.
        for (const SupportMemberProfile* profile :
            {&ledger, &tie, &brace, &trackSupport})
        {
            require(
                profile->outerDimensions.x != profile->outerDimensions.y,
                "non-post roles must be non-square rectangles");
        }
        require(
            ledger.outerDimensions != tie.outerDimensions
                && tie.outerDimensions != brace.outerDimensions
                && brace.outerDimensions != trackSupport.outerDimensions
                && ledger.outerDimensions != trackSupport.outerDimensions,
            "each role must carry a distinct section, not one shared square");

        // Heaviest reads first: posts largest, braces smallest.
        const double postArea =
            post.outerDimensions.x * post.outerDimensions.y;
        const double braceArea =
            brace.outerDimensions.x * brace.outerDimensions.y;
        require(
            postArea > ledger.outerDimensions.x * ledger.outerDimensions.y
                && postArea > tie.outerDimensions.x * tie.outerDimensions.y
                && postArea > trackSupport.outerDimensions.x
                        * trackSupport.outerDimensions.y
                && postArea > braceArea,
            "primary posts must be the heaviest section");
        require(
            braceArea < ledger.outerDimensions.x * ledger.outerDimensions.y
                && braceArea
                    < tie.outerDimensions.x * tie.outerDimensions.y,
            "braces must be the lightest secondary section");

        bool rejected = false;
        try
        {
            static_cast<void>(
                timberProfileForRole(SupportMemberRole::Brace, 0.0));
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected, "non-positive member sizes must be rejected.");

        rejected = false;
        try
        {
            validateSupportMemberRole(
                static_cast<SupportMemberRole>(255));
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected, "unknown member roles must be rejected.");
    }

    void manualMembersDefaultToUnspecified()
    {
        SupportCollection collection;
        const SupportStructureId structureId =
            createSupportStructure(collection, "Manual");
        const SupportElementId base =
            createSupportNode(collection, structureId, {0.0, 0.0, 0.0});
        const SupportElementId top =
            createSupportNode(collection, structureId, {0.0, 0.0, 5.0});
        const SupportElementId memberId = createSupportMember(
            collection, structureId, base, top);
        require(
            collection.structures.front().members.front().role
                == SupportMemberRole::Unspecified,
            "manually authored members default to Unspecified");
        validateSupportCollection(collection);

        const SupportElementId secondBase = createSupportNode(
            collection, structureId, {2.0, 0.0, 0.0});
        const SupportElementId secondTop = createSupportNode(
            collection, structureId, {2.0, 0.0, 5.0});
        const SupportMemberProfile braceProfile = timberProfileForRole(
            SupportMemberRole::Brace, 0.2);
        const SupportElementId braceId = createSupportMember(
            collection,
            structureId,
            secondBase,
            secondTop,
            braceProfile,
            SupportMemberRole::Brace);
        require(
            collection.structures.front().members.back().role
                == SupportMemberRole::Brace,
            "an explicit member role must be stored");
        static_cast<void>(memberId);
        static_cast<void>(braceId);
    }

    [[nodiscard]] const SupportStructure& generatedHybridOneStory()
    {
        static AuthoredTrack cached = [] {
            AuthoredTrack track = elevatedTrack();
            WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0, 0.0, -0.5,
                0.2, true};
            recipe.family = TimberSupportFamily::HybridTimberLattice;
            recipe.storyHeight = 24.0;
            static_cast<void>(generateWoodenSupportRun(track, recipe));
            return track;
        }();
        return cached.supports().structures.front();
    }

    void hybridOneStoryAssignsEveryRole()
    {
        AuthoredTrack track = elevatedTrack();
        WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0, 0.0, -0.5, 0.2,
            true};
        recipe.family = TimberSupportFamily::HybridTimberLattice;
        recipe.storyHeight = 24.0;
        static_cast<void>(generateWoodenSupportRun(track, recipe));
        const SupportStructure& structure =
            track.supports().structures.front();

        auto count = [&](const SupportMemberRole role) {
            return std::count_if(
                structure.members.begin(),
                structure.members.end(),
                [role](const SupportMember& member) {
                    return member.role == role;
                });
        };
        require(
            count(SupportMemberRole::Unspecified) == 0,
            "every generated member must carry an explicit role");
        require(count(SupportMemberRole::PrimaryPost) > 0,
            "a hybrid run needs primary posts");
        require(count(SupportMemberRole::LedgerCap) > 0,
            "a hybrid run needs ledgers/caps");
        require(count(SupportMemberRole::LongitudinalTie) > 0,
            "a hybrid run needs longitudinal ties");
        require(count(SupportMemberRole::Brace) > 0,
            "a hybrid run needs braces");
        require(count(SupportMemberRole::TrackSupport) == 0,
            "Hybrid upper ties must not claim an unestablished track-support function");

        // Hybrid bents resolve sections from the Hybrid-only hierarchy
        // (broader caps, lighter ties, slenderest braces), so they are
        // checked relationally here; the dedicated Hybrid accuracy suite
        // pins the exact archetype behavior.
        for (const SupportMember& member : structure.members)
        {
            validateSupportMemberProfile(member.profile);
            require(
                member.profile.shape
                    == SupportMemberProfileShape::Rectangular,
                "every hybrid timber section must stay rectangular");
            if (member.role == SupportMemberRole::PrimaryPost)
            {
                require(
                    member.profile.outerDimensions.x
                            == recipe.memberSize
                        && member.profile.outerDimensions.y
                            == recipe.memberSize,
                    "hybrid posts must keep the nominal square section");
            }
            else
            {
                require(
                    member.profile.outerDimensions.x
                        != member.profile.outerDimensions.y,
                    "hybrid secondary members must be non-square rectangles");
            }
        }

        // Determinism includes roles: identical inputs agree exactly.
        AuthoredTrack second = elevatedTrack();
        static_cast<void>(generateWoodenSupportRun(second, recipe));
        require(
            second.supports() == track.supports(),
            "identical generator inputs must produce identical roles");
        static_cast<void>(generatedHybridOneStory());
    }

    void allFamiliesCarryExplicitRoles()
    {
        const TimberSupportFamily families[] = {
            TimberSupportFamily::TraditionalTimberBent,
            TimberSupportFamily::ModernTwisterTimber,
            TimberSupportFamily::PrefabricatedTimberLattice,
            TimberSupportFamily::HybridTimberLattice};
        for (const TimberSupportFamily family : families)
        {
            AuthoredTrack track = elevatedTrack();
            WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0, 0.0, -0.5,
                0.22, true};
            recipe.family = family;
            recipe.storyHeight = 18.0;
            static_cast<void>(generateWoodenSupportRun(track, recipe));
            const SupportStructure& structure =
                track.supports().structures.front();
            for (const SupportMember& member : structure.members)
            {
                validateSupportMemberRole(member.role);
                validateSupportMemberOrientation(member.orientation);
                require(
                    member.role != SupportMemberRole::Unspecified,
                    "no generated family may leave a member Unspecified");
                if (family == TimberSupportFamily::HybridTimberLattice)
                {
                    // Hybrid sections come from the Hybrid-only hierarchy;
                    // relational pinning lives in the Hybrid accuracy suite.
                    validateSupportMemberProfile(member.profile);
                    continue;
                }
                require(
                    member.orientation
                        == SupportMemberOrientation::Generic,
                    "non-Hybrid families must keep the generic fallback");
                require(
                    member.profile
                        == timberProfileForRole(
                            member.role, recipe.memberSize),
                    "every family profile must match its role section");
            }
            // All generated sections stay rectangular, so the solid renderer
            // keeps one instanced batch without any renderer change.
            const SupportSolidPresentation presentation =
                buildSupportSolidPresentation(structure);
            require(
                presentation.memberCount() == structure.members.size(),
                "every role-carrying member must still present exactly once");
            require(
                presentation.drawCallCount() <= 2,
                "role sections must not add renderer draw calls");
        }
    }

    void rolesRoundTripAndOldDocumentsDefault()
    {
        AuthoredTrack track = elevatedTrack();
        WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0, 0.0, -0.5, 0.2,
            true};
        recipe.family = TimberSupportFamily::HybridTimberLattice;
        static_cast<void>(generateWoodenSupportRun(track, recipe));

        const std::string serialized = serializeCoasterDocument(track);
        const json document = json::parse(serialized);
        require(
            document["supports"]["structures"][0]["members"][0].contains(
                "role"),
            "generated members must persist their role by stable name");
        require(
            document["supports"]["structures"][0]["members"][0]["role"]
                    .is_string(),
            "member roles must serialize as text, never numeric codes");

        const auto restored = deserializeCoasterDocument(serialized);
        require(restored.has_value(), "a role-carrying document must load");
        require(
            restored->supports() == track.supports(),
            "member roles and sections must round-trip exactly");
        require(
            serializeCoasterDocument(*restored) == serialized,
            "role serialization must be deterministic");

        // M1/M2A documents have no role field at all.
        json legacy = document;
        for (auto& member :
            legacy["supports"]["structures"][0]["members"])
        {
            member.erase("role");
        }
        const auto legacyRestored =
            deserializeCoasterDocument(legacy.dump());
        require(
            legacyRestored.has_value(),
            "a document without roles must still load");
        for (const SupportMember& member :
            legacyRestored->supports().structures.front().members)
        {
            require(
                member.role == SupportMemberRole::Unspecified,
                "absent roles must default to Unspecified");
        }

        // Manual Unspecified members stay compact.
        SupportCollection manual;
        const SupportStructureId structureId =
            createSupportStructure(manual, "Manual");
        const SupportElementId base = createSupportNode(
            manual, structureId, {0.0, 0.0, 0.0});
        const SupportElementId top = createSupportNode(
            manual, structureId, {0.0, 0.0, 5.0});
        static_cast<void>(
            createSupportMember(manual, structureId, base, top));
        AuthoredTrack manualTrack = createNewDocument();
        manualTrack.setSupports(manual);
        const json manualDocument =
            json::parse(serializeCoasterDocument(manualTrack));
        require(
            !manualDocument["supports"]["structures"][0]["members"][0]
                 .contains("role"),
            "Unspecified roles must be omitted so manual documents keep "
            "their byte layout");

        json unknown = document;
        unknown["supports"]["structures"][0]["members"][0]["role"] =
            "InventedRole";
        require(
            !deserializeCoasterDocument(unknown.dump()).has_value(),
            "unknown role names must be rejected");
        json numeric = document;
        numeric["supports"]["structures"][0]["members"][0]["role"] = 2;
        require(
            !deserializeCoasterDocument(numeric.dump()).has_value(),
            "numeric roles must be rejected");
    }
}

int main()
{
    try
    {
        roleProfilesAreDistinctRectangles();
        manualMembersDefaultToUnspecified();
        hybridOneStoryAssignsEveryRole();
        allFamiliesCarryExplicitRoles();
        rolesRoundTripAndOldDocumentsDefault();
    }
    catch (const std::exception& error)
    {
        std::cerr << "Support member role test failure: " << error.what()
                  << '\n';
        return 1;
    }

    std::cout << "Support member role tests passed.\n";
    return 0;
}
