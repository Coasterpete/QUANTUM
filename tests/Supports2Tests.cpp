#include <quantum/editor/ApplicationService.hpp>
#include <quantum/editor/InfrastructurePresentation.hpp>
#include <quantum/editor/SupportPicking.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/WoodenSupportGenerator.hpp>
#include <quantum/renderer/SupportSolidGeometry.hpp>
#include <nlohmann/json.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numbers>
#include <stdexcept>

namespace
{
    using namespace quantum;
    using namespace quantum::coaster;
    unsigned checks = 0;
    void require(bool condition, const char* message)
    { ++checks; if (!condition) throw std::runtime_error(message); }
    template<class F> void rejects(F action)
    { bool rejected=false; try { action(); } catch (const std::exception&) { rejected=true; } require(rejected,"Invalid authored data must be rejected."); }
    std::string saved(const editor::ApplicationService& app) { return serializeCoasterDocument(app.track()); }
    AuthoredTrack track()
    {
        auto t = createNewDocument(); t.section(0).length = 60;
        return t;
    }
    AuthoredTrack curvedTrack()
    {
        auto t = track();
        auto& rates = t.section(0).rateProfileRegion().rateProfiles;
        for (auto* channel : {&rates.pitch,&rates.yaw,&rates.roll})
            channel->segments[0].transition.transitionType = math::TransitionType::Smootherstep;
        rates.pitch.segments[0].transition.valueBegin = -.008;
        rates.pitch.segments[0].transition.valueEnd = .006;
        rates.yaw.segments[0].transition.valueBegin = .006;
        rates.yaw.segments[0].transition.valueEnd = .025;
        rates.roll.segments[0].transition.valueBegin = .012;
        rates.roll.segments[0].transition.valueEnd = -.009;
        return t;
    }
    template<class F> void history(editor::ApplicationService& app, F action)
    {
        const auto before = saved(app); const auto revision = app.revision();
        action(); const auto after = saved(app);
        require(before != after && app.revision() == revision+1,"One authored gesture must publish one revision.");
        app.restoreHistory(true); require(saved(app)==before,"Undo must restore exact authored state.");
        app.restoreHistory(false); require(saved(app)==after,"Redo must restore exact authored state.");
        const auto reload = deserializeCoasterDocument(after);
        require(reload.has_value() && serializeCoasterDocument(*reload)==after,"Save/reload must preserve all authored values.");
    }
    SupportStructure single(SupportMemberProfile profile, glm::dvec3 end={20,0,0})
    {
        SupportStructure s; s.id=1; s.name="Profile"; s.nextElementId=4;
        s.nodes={{1,{0,0,0}},{2,end}}; s.members={{3,1,2,profile}};
        return s;
    }
    void materialsAndProfiles()
    {
        auto s = single({SupportMemberProfileShape::Rectangular,{.3,.4},0});
        for (auto material : {SupportMaterialKind::Timber,SupportMaterialKind::PaintedSteel,
            SupportMaterialKind::GalvanizedSteel,SupportMaterialKind::Concrete})
        {
            s.surface=defaultSupportSurface(material);
            const auto solid=buildSupportSolidPresentation(s);
            require(solid.batches[0].surface.material==material,"Rectangular shape must permit every material.");
            require(solid.batches[0].instances[0].transform==buildSupportSolidPresentation(single(s.members[0].profile)).batches[0].instances[0].transform,
                "Changing material must preserve member geometry.");
        }
        s.style=SupportStyleFamily::TubularSteel; s.surface.reset();
        require(resolveSupportSurface(s,s.members[0]).material==SupportMaterialKind::PaintedSteel,"Steel family has steel defaults.");
        s.members[0].surface=defaultSupportSurface(SupportMaterialKind::Concrete);
        s.surface=defaultSupportSurface(SupportMaterialKind::Timber);
        require(resolveSupportSurface(s,s.members[0]).material==SupportMaterialKind::Concrete,"Member material overrides structure and family.");
        s.members[0].surface.reset(); require(resolveSupportSurface(s,s.members[0]).material==SupportMaterialKind::Timber,"Structure overrides family.");
        require(resolveSupportProfile(s,s.members[0]).shape==SupportMemberProfileShape::Rectangular,"Explicit profiles survive family changes.");
        s.members[0].useStyleProfile=true;
        require(resolveSupportProfile(s,s.members[0]).shape==SupportMemberProfileShape::Circular,"Opted-in member follows family profile.");
        for (unsigned family=0;family<=5;++family)
            for (auto role : {SupportMemberRole::PrimaryPost,SupportMemberRole::LedgerCap,SupportMemberRole::Brace})
                validateSupportMemberProfile(supportStyleProfile(static_cast<SupportStyleFamily>(family),role));
        rejects([] { validateSupportMemberProfile({SupportMemberProfileShape::Hss,{.3,.4},0}); });
        rejects([] { validateSupportMemberProfile({SupportMemberProfileShape::ISection,{.3,.4},.16}); });
        rejects([] { auto surface=SupportSurface{};surface.repeatMeters.x=0;validateSupportSurface(surface); });
        rejects([] { static_cast<void>(deserializeSupportSurface("{\"material\":\"Unknown\"}")); });
        require(deserializeSupportSurface(serializeSupportSurface(defaultSupportSurface(SupportMaterialKind::GalvanizedSteel)))
            == defaultSupportSurface(SupportMaterialKind::GalvanizedSteel),"Surface schema is deterministic.");
        s.members[0].surface.reset(); s.surface=defaultSupportSurface(SupportMaterialKind::Timber);
        require(resolveSupportSurface(s,s.members[0],.5).repeatMeters==s.surface->repeatMeters,
            "New authored SI repeats must not change with document scale.");
        s.surface.reset();s.style=SupportStyleFamily::TraditionalTimber;s.appearance=SupportAppearance{};
        require(std::abs(resolveSupportSurface(s,s.members[0],.5).repeatMeters.x-.5F)<1e-6,
            "Legacy Core-unit appearance repeats must convert once to SI.");
        s.appearance->textureScale=.0001;
        const auto legacy=resolveSupportSurface(s,s.members[0],.5);
        validateSupportSurface(legacy);
        require(std::abs(legacy.repeatMeters.x-.00005F)<1e-10,
            "Positive legacy scales remain valid after SI conversion, including submillimeter repeats.");
    }
    void physicalUvs()
    {
        const glm::dvec2 repeat{2,.5};
        for (double length : {2.0,20.0,1.1e-6})
        {
            const auto a=supportSurfaceUv({-length*.5,-.15,.2},{0,0,1},{length,.3,.4},repeat,false);
            const auto b=supportSurfaceUv({length*.5,-.15,.2},{0,0,1},{length,.3,.4},repeat,false);
            require(std::abs((b.x-a.x)/length-.5)<1e-8,"Longitudinal repeat density must be length independent, including very short members.");
            require(std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(b.x)&&std::isfinite(b.y),"All short member UV coordinates must be finite.");
        }
        for (const auto& end : {glm::dvec3{20,0,0},glm::dvec3{0,0,20},glm::dvec3{12,-5,9},glm::dvec3{-3,14,11}})
        {
            const auto f=resolveSupportMemberFrame({0,0,0},end);
            const auto position=f.origin+f.axisX*1.0+f.axisY*.1+f.axisZ*.2;
            const auto delta=position-f.origin;
            const glm::dvec3 local{glm::dot(delta,f.axisX),glm::dot(delta,f.axisY),glm::dot(delta,f.axisZ)};
            const auto uv=supportSurfaceUv(local,{0,0,1},{f.length,.3,.4},repeat,false);
            require(glm::length(uv-glm::dvec2{.5,.2})<1e-12,"Rotated posts/diagonal braces must map in their local frame.");
        }
        const auto box=renderer::createSupportSolidBoxMesh();
        for (const auto& v : box.vertices)
        {
            if (std::abs(v.normal.x)>.5) continue;
            const glm::dvec3 n{v.normal};
            const auto a=supportSurfaceUv({0,0,0},n,{2,.3,.4},{1,1},false);
            const auto p=glm::cross(n,glm::dvec3{1,0,0});
            const auto b=supportSurfaceUv(p,n,{2,.3,.4},{1,1},false);
            require(std::abs(b.y-a.y-1)<1e-12,"Opposite box faces must preserve tangent-space handedness.");
        }
        const auto round=renderer::createSupportSolidCylinderMesh();
        const auto seamA=supportSurfaceUv({0,.25,0},{0,1,0},{2,.5,.5},{1,1},true,0);
        const auto seamB=supportSurfaceUv({0,.25,0},{0,1,0},{2,.5,.5},{1,1},true,1);
        require(std::abs(std::abs(seamB.y-seamA.y)-std::numbers::pi*.5)<1e-12,"Cylindrical seam span must equal circumference.");
        require(round.vertices[0].uv.y==0 && round.vertices[32].uv.y==1,"Unit cylinder must duplicate the unwrap seam with distinct coordinates.");
        const auto concrete=supportSurfaceUv({3,.4,.4},{0,0,1},{10,.8,.8},{1,1},false);
        require(concrete==glm::dvec2{3,.4},"Concrete uses predictable physical mapping.");
        rejects([] { static_cast<void>(supportSurfaceUv({0,0,0},{0,0,1},{0,.3,.4},{1,1},false)); });
    }
    void shapesAndTreatments()
    {
        for (auto shape : {SupportMemberProfileShape::Circular,SupportMemberProfileShape::Hss,SupportMemberProfileShape::ISection})
        {
            auto s=single({shape,{.4,.4},.025});s.surface=defaultSupportSurface(SupportMaterialKind::PaintedSteel);
            const auto solid=buildSupportSolidPresentation(s);
            require(solid.memberCount()==(shape==SupportMemberProfileShape::Circular ? 1u : shape==SupportMemberProfileShape::Hss ? 4u : 3u),
                "CHS/HSS/I sections must produce bounded, meaningful geometry.");
            if (shape==SupportMemberProfileShape::Circular)
            {
                require(solid.batches[0].mesh==SupportMemberMeshKind::CircularTube,"Positive circular wall thickness must select a hollow tube.");
                require(std::abs(solid.batches[0].instances[0].section.x-.875F)<1e-6,"Tube bore must match wall thickness.");
            }
            for (const auto& b : solid.batches) for (const auto& i : b.instances)
                require(glm::determinant(glm::mat3{i.transform})>0,"Every profile component needs a finite positive basis.");
        }
        const auto tube=renderer::createSupportSolidCylinderMesh(16,true);
        require(tube.vertices.size()==136 && tube.triangleIndices.size()==384,"Shared tube mesh must remain modest.");
        for (std::size_t tri=0;tri<tube.triangleIndices.size();tri+=3)
        {
            auto vertex=[&](std::uint32_t index) { auto p=tube.vertices[index].position; if(tube.vertices[index].inner>.5) {p.y*=.8F;p.z*=.8F;}return p; };
            const auto a=tube.triangleIndices[tri],b=tube.triangleIndices[tri+1],c=tube.triangleIndices[tri+2];
            const auto normal=glm::cross(vertex(b)-vertex(a),vertex(c)-vertex(a));
            require(glm::dot(normal,tube.vertices[a].normal)>0,"Tube walls and annular caps must face outward from their material.");
        }
        auto s=single({SupportMemberProfileShape::Rectangular,{.3,.4},0});
        s.nodes[0].foundation=Foundation{};s.nodes[1].trackAttachment=TrackAttachment{20,0,0};
        for (unsigned treatment=1;treatment<=8;++treatment)
        {
            s.members[0].startConnection.reset();s.members[0].endConnection.reset();
            auto connection=SupportMemberEndConnection{static_cast<SupportMemberEndTreatment>(treatment)};
            const bool base=treatment>=7;
            (base ? s.members[0].startConnection : s.members[0].endConnection)=connection;
            require(buildSupportSolidPresentation(s).memberCount()>1,"Every non-mitered treatment must generate visible geometry.");
        }
        SupportCollection collection{{s},2};
        s.members[0].endConnection=SupportMemberEndConnection{SupportMemberEndTreatment::Base};collection.structures[0]=s;
        rejects([&] { validateSupportCollection(collection); });
        for (unsigned kind=0;kind<4;++kind)
        {
            s.nodes[0].foundation=Foundation{static_cast<SupportFoundationKind>(kind),{1,1,2}};
            const auto solid=buildSupportSolidPresentation(s);
            require(solid.foundations[0].kind==static_cast<SupportFoundationKind>(kind)&&solid.foundations[0].padDepth==2,
                "Foundation kind and explicit dimensions must reach presentation.");
            const auto parts=renderer::createSupportFoundationInstances(solid.foundations[0]);
            require(parts.size()==(kind==1 ? 2u : 1u),"Pedestals must have two tiers; footings/blocks/piers one body.");
            for (const auto& part : parts) require(glm::determinant(glm::mat3{part.transform})>0,
                "Concrete foundation geometry must have a positive, nondegenerate basis.");
            require(parts[0].section.y==(kind==2 ? 1 : 0),"Only pier foundations use round columns.");
        }
    }
    void authoredHistoryAndFrames()
    {
        editor::ApplicationService app{track()};
        editor::SupportEdit request{editor::SupportOperation::GenerateFrame};request.frame.station=10;
        std::uint32_t id=0;
        history(app,[&] {id=app.editSupport(request);});
        require(app.track().supports().structures[0].members.size()==4 && app.track().supports().structures[0].nodes.size()==4,
            "Generated frame must be an inspectable graph.");
        const auto memberId=app.track().supports().structures[0].members[0].id;
        editor::SupportEdit material{editor::SupportOperation::SetMemberSurface};material.structureId=id;material.elementId=memberId;
        material.surface=defaultSupportSurface(SupportMaterialKind::GalvanizedSteel);
        history(app,[&]{static_cast<void>(app.editSupport(material));});
        auto scale=*material.surface;scale.repeatMeters={3,.75F};material.surface=scale;
        history(app,[&]{static_cast<void>(app.editSupport(material));});
        editor::SupportEdit style{editor::SupportOperation::SetStyle};style.structureId=id;style.style=SupportStyleFamily::TubularSteel;
        history(app,[&]{static_cast<void>(app.editSupport(style));});
        require(app.track().supports().structures[0].members[0].surface==scale,"Family edits preserve member material overrides.");
        request.structureId=id;request.frame.width=5;
        history(app,[&]{static_cast<void>(app.editSupport(request));});
        require(app.track().supports().structures[0].members[0].id==memberId
            &&app.track().supports().structures[0].members[0].surface==scale,"Parameter regeneration preserves identity and overrides.");
        editor::SupportEdit profile{editor::SupportOperation::SetProfile};profile.structureId=id;profile.elementId=memberId;
        profile.profile={SupportMemberProfileShape::ISection,{.3,.5},.02};
        history(app,[&]{static_cast<void>(app.editSupport(profile));});
        request.frame.post.outerDimensions={.5,.5};
        history(app,[&]{static_cast<void>(app.editSupport(request));});
        require(app.track().supports().structures[0].members[0].profile==profile.profile,"Regeneration preserves intentional profile overrides.");
        // An explicit choice equal to the old default is still an override.
        profile.elementId=app.track().supports().structures[0].members[1].id;
        profile.profile=app.track().supports().structures[0].members[1].profile;
        history(app,[&]{static_cast<void>(app.editSupport(profile));});
        request.frame.post.outerDimensions={.6,.6};
        history(app,[&]{static_cast<void>(app.editSupport(request));});
        require(app.track().supports().structures[0].members[1].profile==profile.profile,
            "Explicit profiles equal to a recipe default must survive regeneration.");
        editor::SupportEdit remove{editor::SupportOperation::DeleteStructure};remove.structureId=id;
        history(app,[&]{static_cast<void>(app.editSupport(remove));});
        app.restoreHistory(true);
        const auto before=app.supports().nodes;
        history(app,[&]{app.setTrackStartPosition({7,-4,3});});
        for (std::size_t i=0;i<before.size();++i)
            require(glm::length(app.supports().nodes[i].position-before[i].position-(i<2 ? glm::dvec3{0} : glm::dvec3{7,-4,3}))<1e-8,
                "Attached frame nodes follow Track Start; world foundations remain fixed.");
    }
    void infrastructureAndAccess()
    {
        auto t=track();
        auto pose=t.startPose();pose.orientation=glm::angleAxis(.35,glm::dvec3{1,0,0});t.setStartPose(pose);
        editor::ApplicationService app{t};
        const auto gateId=app.createDevice(TrackDeviceKind::Access);
        auto access=AccessConfiguration{};access.initiallyLocked=false;access.widthMeters=1.5;access.heightMeters=1.4;
        history(app,[&]{app.setHardwareConfiguration(gateId,access);});
        TrackInfrastructure cat;cat.id=1;cat.name="Catwalk";cat.startStationMeters=5;cat.endStationMeters=20;cat.accessGates={gateId};
        history(app,[&]{app.setInfrastructure({cat},2);});
        const auto solids=editor::createInfrastructurePresentation(app.track(),app.centerline().visualization().samples);
        require(!solids[0].solid.empty() && solids[0].solid.memberCount()>30,"Catwalk must generate decks, framing and bounded guardrails.");
        const auto& deck=solids[0].solid.batches[0].instances[0];
        require(glm::dot(glm::normalize(glm::dvec3{deck.transform[2]}),coaster::startPoseRiderFrame(pose).up)>.999,
            "Banked catwalk deck normal must follow evaluated track up.");
        const auto before=glm::dvec3{deck.transform[3]};
        history(app,[&]{app.setTrackStartPosition({4,3,2});});
        const auto moved=editor::createInfrastructurePresentation(app.track(),app.centerline().visualization().samples);
        require(glm::length(glm::dvec3{moved[0].solid.batches[0].instances[0].transform[3]}-before-glm::dvec3{4,3,2})<1e-5,
            "Track-relative catwalk must follow Track Start relocation.");
        auto platform=cat;platform.id=2;platform.kind=InfrastructureKind::Platform;platform.name="Platform";
        platform.worldPlacement=true;platform.worldStartMeters={0,5,0};platform.worldEndMeters={4,5,0};platform.widthMeters=2;
        auto stairs=platform;stairs.id=3;stairs.kind=InfrastructureKind::Stairs;stairs.name="Stairs";stairs.worldEndMeters={4,5,2};stairs.stepRiseMeters=.2;
        auto landing=platform;landing.id=4;landing.kind=InfrastructureKind::Landing;landing.name="Landing";
        auto rail=cat;rail.id=5;rail.kind=InfrastructureKind::Guardrail;rail.name="Guardrail";rail.railings.inner=true;
        history(app,[&]{app.setInfrastructure({cat,platform,stairs,landing,rail},6);});
        const auto objects=editor::createInfrastructurePresentation(app.track(),app.centerline().visualization().samples);
        require(objects.size()==5&&!objects[2].solid.empty(),"Platforms, stairs, landings and standalone guardrails must present.");
        const auto fixed=objects[1].solid.batches[0].instances[0].transform;
        history(app,[&]{app.setTrackStartPosition({8,3,2});});
        require(editor::createInfrastructurePresentation(app.track(),app.centerline().visualization().samples)[1].solid.batches[0].instances[0].transform==fixed,
            "World infrastructure must stay fixed when Track Start moves.");
        auto invalid=cat;invalid.accessGates={gateId+999};
        rejects([&]{app.setInfrastructure({invalid},6);});
        auto closed=editor::createAccessPresentation(app.track(),app.centerline().visualization().samples);
        physics::HardwareRuntime runtime;runtime.id=gateId;
        physics::AccessRuntime state;state.fraction=.5;state.locked=false;runtime.mechanism=state;
        auto moving=editor::createAccessPresentation(app.track(),app.centerline().visualization().samples,std::span{&runtime,1});
        const auto& a=closed[0].solid.batches[0].instances;const auto& b=moving[0].solid.batches[0].instances;
        require(a.size()==7&&a[0]==b[0]&&a[1]==b[1]&&a[2]!=b[2],"Gate leaf must move with runtime fraction while posts stay fixed.");
        access.motion=AccessMotion::Slide;access.infill=AccessInfill::Solid;
        history(app,[&]{app.setHardwareConfiguration(gateId,access);});
        const auto slideClosed=editor::createAccessPresentation(app.track(),app.centerline().visualization().samples);
        const auto slideMoving=editor::createAccessPresentation(app.track(),app.centerline().visualization().samples,std::span{&runtime,1});
        require(glm::length(glm::dvec3{slideMoving[0].solid.batches[0].instances[2].transform[3]}
            -glm::dvec3{slideClosed[0].solid.batches[0].instances[2].transform[3]})>.4,"Sliding gate must use existing runtime travel fraction.");
        const auto center=glm::dvec3{objects[1].solid.batches[0].instances[0].transform[3]};
        auto hit=editor::pickInfrastructureSolid(std::span{&objects[1],1},center+glm::dvec3{0,0,5},{0,0,-1});
        require(hit&&hit->id==2&&!hit->access,"Infrastructure picking must return stable authored ID.");
    }
    void benchmark()
    {
        for (unsigned count : {100u,1000u})
        {
            SupportStructure s;s.id=1;s.name="Support-heavy";s.nextElementId=count*3+1;
            for (unsigned i=0;i<count;++i)
            {
                s.nodes.push_back({i*3+1,{static_cast<double>(i),0,0}});
                s.nodes.push_back({i*3+2,{static_cast<double>(i),0,20}});
                s.members.push_back({i*3+3,i*3+1,i*3+2,{SupportMemberProfileShape::Rectangular,{.3,.3},0}});
            }
            const auto start=std::chrono::steady_clock::now();
            const auto solid=buildSupportSolidPresentation(s);
            const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
            require(solid.memberCount()==count&&solid.drawCallCount()==1,"Support-heavy batching must stay linear and use one draw.");
            std::cout<<"Members "<<count<<": "<<ms<<" ms, 24 shared vertices / 36 shared indices, "
                <<count*24<<" expanded vertices / "<<count*36<<" expanded indices, "<<count*sizeof(SupportMemberInstance)<<" instance bytes\n";
        }
        editor::ApplicationService app{track()}; TrackInfrastructure cat;cat.id=1;cat.name="Perf catwalk";cat.startStationMeters=0;cat.endStationMeters=50;
        app.setInfrastructure({cat},2);
        const auto start=std::chrono::steady_clock::now();
        const auto objects=editor::createInfrastructurePresentation(app.track(),app.centerline().visualization().samples);
        const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        const auto count=objects[0].solid.memberCount();
        std::cout<<"50 m catwalk: "<<ms<<" ms, "<<count<<" instances, "<<count*24<<" expanded vertices / "<<count*36<<" expanded indices\n";
    }
    void curvedInfrastructure()
    {
        editor::ApplicationService app{curvedTrack()};
        TrackInfrastructure cat;cat.id=1;cat.name="Transition catwalk";cat.startStationMeters=0;cat.endStationMeters=55;
        app.setInfrastructure({cat},2);
        const auto solids=editor::createInfrastructurePresentation(app.track(),app.centerline().visualization().samples);
        const auto& decks=solids[0].solid.batches[0].instances;
        require(decks.size()>70,"Curved catwalk must be sampled along its interval.");
        for (std::size_t index=0;index<decks.size();index+=9)
        {
            const auto station=55.0*(index+.5)/decks.size();
            const auto evaluated=resolveSupportTrackAttachment(app.centerline().visualization().samples,
                {station,cat.lateralOffsetMeters,cat.verticalOffsetMeters});
            require(glm::dot(glm::normalize(glm::dvec3{decks[index].transform[2]}),evaluated.frame.up)>.998,
                "Deck normals must follow native grade, banking and curvature transitions.");
            require(glm::length(glm::dvec3{decks[index].transform[0]})<.80,
                "Track-following deck chords must remain bounded.");
        }
        require(glm::dot(glm::normalize(glm::dvec3{decks.front().transform[0]}),
            glm::normalize(glm::dvec3{decks.back().transform[0]}))<.95,"Curved catwalk must not collapse to one straight rectangle.");
        auto solidDeck=cat;solidDeck.deck=DeckStyle::Solid;
        history(app,[&]{app.setInfrastructure({solidDeck},2);});
        require(editor::createInfrastructurePresentation(app.track(),app.centerline().visualization().samples)[0]
            .solid.batches[0].instances[0].section.z==0,"Solid deck must disable the grating pattern.");
        history(app,[&]{app.setInfrastructure({},2);});
        app.restoreHistory(true);
        require(app.track().trackDevices().infrastructure[0].id==cat.id,"Deleted infrastructure Undo restores its stable ID.");
    }
    void writeDemo(const char* path)
    {
        auto t=curvedTrack();
        for (unsigned family=0;family<5;++family)
        {
            const auto style=static_cast<SupportStyleFamily>(family);
            SupportFrameRecipe recipe;recipe.station=5+family*10;
            recipe.post=supportStyleProfile(style,SupportMemberRole::PrimaryPost);
            recipe.beam=supportStyleProfile(style,SupportMemberRole::LedgerCap);
            recipe.brace=supportStyleProfile(style,SupportMemberRole::Brace);
            static_cast<void>(generateSupportFrame(t,style,recipe));
        }
        auto device=makeTrackDevice(TrackDeviceKind::Access);device.startStationMeters=15;device.endStationMeters=16;
        auto access=AccessConfiguration{};access.initiallyLocked=false;access.verticalOffsetMeters=-.25;access.lateralOffsetMeters=1.1;
        device.configuration=access;const auto gate=t.addTrackDevice(device);
        auto devices=t.trackDevices();
        TrackInfrastructure cat;cat.id=1;cat.name="Bank-following catwalk";cat.startStationMeters=3;cat.endStationMeters=48;cat.verticalOffsetMeters=-.25;cat.accessGates={gate};
        auto platform=cat;platform.id=2;platform.name="Maintenance platform";platform.kind=InfrastructureKind::Platform;
        platform.worldPlacement=true;platform.worldStartMeters={15,2,-.25};platform.worldEndMeters={19,2,-.25};platform.widthMeters=2;
        auto stairs=platform;stairs.id=3;stairs.name="Ground stairs";stairs.kind=InfrastructureKind::Stairs;stairs.worldStartMeters={19,2,-.25};stairs.worldEndMeters={24,2,-5};
        auto landing=platform;landing.id=4;landing.name="Ground landing";landing.kind=InfrastructureKind::Landing;landing.worldStartMeters={24,2,-5};landing.worldEndMeters={26,2,-5};
        devices.infrastructure={cat,platform,stairs,landing};devices.nextInfrastructureId=5;t.setTrackDevices(devices);
        std::ofstream file(path);file<<serializeCoasterDocument(t);if(!file)throw std::runtime_error("Could not write support acceptance document.");
    }
}
int main(int argc,char** argv)
{
    try
    {
        if(argc==3&&std::string_view{argv[1]}=="--write-demo") {writeDemo(argv[2]);return 0;}
        materialsAndProfiles();physicalUvs();shapesAndTreatments();authoredHistoryAndFrames();infrastructureAndAccess();curvedInfrastructure();benchmark();
        std::cout<<"Supports 2: "<<checks<<" checks passed.\n";return 0;
    }
    catch(const std::exception& error) {std::cerr<<error.what()<<"\n";return 1;}
}
