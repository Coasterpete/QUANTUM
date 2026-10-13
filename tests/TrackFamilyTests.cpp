#include <quantum/coaster/TrackConfiguration.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/WoodenSupportGenerator.hpp>
#include <quantum/editor/ApplicationService.hpp>
#include <quantum/editor/DocumentHistory.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/renderer/StaticMeshAssets.hpp>
#include <nlohmann/json.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/geometric.hpp>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>

namespace
{
    using namespace quantum;
    using namespace coaster;
    using namespace editor;
    using Json=nlohmann::json;
    unsigned checks=0;
    void require(bool condition,const char* message)
    { ++checks; if (!condition) throw std::runtime_error(message); }
    template<class F> void rejects(F&& action,const char* message)
    { bool rejected=false; try {action();} catch(const std::exception&) {rejected=true;} require(rejected,message); }
    std::vector<RiderLocalGeometryState> samples(double length=60, double roll=0)
    {
        const auto q=glm::angleAxis(roll,glm::dvec3{1,0,0});
        const geometry::CurveFrame frame{{1,0,0},q*glm::dvec3{0,1,0},q*glm::dvec3{0,0,1}};
        std::vector<RiderLocalGeometryState> out;
        for(double d=0;d<length;d+=.5) out.push_back({d,{d,0,10},frame});
        out.push_back({length,{length,0,10},frame});
        return out;
    }
    void geometryValid(const RenderableTrack& mesh)
    {
        require(!mesh.continuousMesh.vertices.empty(),"Every selectable type must produce visible geometry");
        for(const auto& v:mesh.continuousMesh.vertices)
        {
            require(std::isfinite(v.position.x)&&std::isfinite(v.position.y)&&std::isfinite(v.position.z),"finite component vertices");
            require(std::abs(glm::length(v.normal)-1)<2e-5,"unit component normals");
        }
        for(const auto& sub:mesh.continuousMesh.submeshes)
        {
            require(sub.materialIndex<mesh.materials.size(),"valid component material index");
            require(sub.firstIndex+sub.indexCount<=mesh.continuousMesh.triangleIndices.size(),"valid submesh range");
        }
        for(std::size_t i=0;i<mesh.continuousMesh.triangleIndices.size();i+=3)
        {
            const auto& a=mesh.continuousMesh.vertices.at(mesh.continuousMesh.triangleIndices[i]);
            const auto& b=mesh.continuousMesh.vertices.at(mesh.continuousMesh.triangleIndices[i+1]);
            const auto& c=mesh.continuousMesh.vertices.at(mesh.continuousMesh.triangleIndices[i+2]);
            const auto facing=glm::dot(glm::cross(b.position-a.position,c.position-a.position),a.normal+b.normal+c.normal);
            if(facing < -1e-5f) throw std::runtime_error("outward triangle winding: triangle "+std::to_string(i/3)+" facing "+std::to_string(facing));
            require(facing>=-1e-5f,"outward triangle winding");
        }
    }
    AuthoredTrack fixture(std::string_view type)
    {
        auto track=createNewDocument();
        track.setLayoutMode(LayoutMode::Shuttle);
        track.setStartPose({{0,0,50},{1,0,0,0}});
        track.section(0)=createRateProfileSection(70);
        track.appendSection();track.section(1)=createRateProfileSection(45);
        for(auto* profile:{&track.section(1).rateProfileRegion().rateProfiles.yaw,&track.section(1).rateProfileRegion().rateProfiles.roll})
            profile->segments.front().transition.valueBegin=profile->segments.front().transition.valueEnd=.022;
        track.appendSection();track.section(2)=createRateProfileSection(100);
        track.section(2).rateProfileRegion().rateProfiles.pitch.segments.front().transition.valueBegin=.04;
        track.section(2).rateProfileRegion().rateProfiles.pitch.segments.front().transition.valueEnd=.04;
        track.applyTrackConfiguration(type);
        return track;
    }
    void catalogAndGeometry()
    {
        const auto straight=samples();
        auto one=createNewDocument();one.section(0)=createRateProfileSection(140);one.applyTrackConfiguration("truss-100");
        auto split=one;split.section(0)=createRateProfileSection(70);split.appendSection();split.section(1)=createRateProfileSection(70);
        const auto oneView=createCenterlineVisualization(one), splitView=createCenterlineVisualization(split);
        const auto& singleMesh=oneView.renderableTrack;const auto& splitMesh=splitView.renderableTrack;
        // Region endpoints may add a solve sample. Compare the repeated
        // geometry in world space rather than requiring identical tessellation.
        const auto splitStyle=one.trackStyle();
        for(std::size_t component=0; component<splitStyle.components.size(); ++component)
        {
            if(splitStyle.components[component].placement==TrackComponentPlacement::Continuous) continue;
            const auto id=static_cast<std::uint32_t>(component+1);
            const auto find=[id](const auto& mesh) {
                return std::find_if(mesh.submeshes.begin(),mesh.submeshes.end(),[id](const auto& sub){return sub.componentId==id;});
            };
            const auto first=find(singleMesh.continuousMesh),second=find(splitMesh.continuousMesh);
            require(first!=singleMesh.continuousMesh.submeshes.end() && second!=splitMesh.continuousMesh.submeshes.end()
                && first->indexCount==second->indexCount,"Region splitting preserves repeated component count");
            for(std::uint32_t index=0; index<first->indexCount; ++index)
            {
                const auto& a=singleMesh.continuousMesh.vertices.at(singleMesh.continuousMesh.triangleIndices[first->firstIndex+index]);
                const auto& b=splitMesh.continuousMesh.vertices.at(splitMesh.continuousMesh.triangleIndices[second->firstIndex+index]);
                require(glm::length(a.position-b.position)<1e-5f && glm::length(a.normal-b.normal)<1e-5f,
                    "Region splitting preserves world-space ties and bracing across the boundary");
            }
        }

        unsigned available=0;
        for(const auto& [id,def]:trackFamilyRegistry().types())
        {
            if(def.reserved || id==modernSteelTrackConfigurationId) continue;
            ++available;
            const auto style=resolveTrackConfiguration(id);
            require(style.typeId==id && style.familyId==def.familyId,"Stable numbered identity");
            const auto mesh=generateRenderableTrack(straight,style);geometryValid(mesh);
            require(mesh.continuousMesh.submeshes.size()<=style.components.size(),"Repeated blockout geometry is batched by component");
            auto track=fixture(id);auto centerline=createCenterlineVisualization(track);geometryValid(centerline.renderableTrack);
            require(centerline.renderableTrack.continuousMesh.submeshes.size()<=style.components.size(),"Uniform component presentation is independent of mathematical Region boundaries");
            auto saved=serializeCoasterDocument(track);auto loaded=deserializeCoasterDocument(saved);
            require(loaded.has_value(),"Family document save/reopen");
            require(serializeCoasterDocument(*loaded)==saved,"Family document lossless round trip");
            std::cout<<id<<": "<<mesh.continuousMesh.vertices.size()<<" vertices, "<<mesh.continuousMesh.submeshes.size()<<" draws\n";
        }
        require(available==17,"All 17 assigned numbered types are selectable");
        require(findTrackConfiguration("wood-300")->reserved,"Wood-300 is reserved");
        rejects([]{static_cast<void>(resolveTrackConfiguration("wood-300"));},"Reserved type cannot be selected");
        require(findTrackConfiguration("box-110")->reference.find("Banshee")!=std::string::npos,"Box-110 is Banshee");
        require(findTrackConfiguration("box-230")->reference.find("Pipeline")!=std::string::npos,"Box-230 is Pipeline");
        for(const auto& id:{"monorail-100","monorail-200"})
            require(resolveTrackConfiguration(id).railCount==1,"Monorail has one running reference");
        auto first=resolveTrackConfiguration("monorail-100"),second=resolveTrackConfiguration("monorail-200");
        require(generateRenderableTrack(straight,first).continuousMesh.vertices.size()!=generateRenderableTrack(straight,second).continuousMesh.vertices.size(),"Distinct monorail structural topology");
        const auto rotated=samples(60,3.14159265358979323846);
        auto style=resolveTrackConfiguration("truss-100");
        auto mesh=generateRenderableTrack(rotated,style);geometryValid(mesh);
        glm::dvec3 center{0};for(unsigned i=0;i<24;++i) center+=glm::dvec3{mesh.continuousMesh.vertices[i].position};center/=24.;
        require(glm::length(center-glm::dvec3{0,.55,10})<1e-6,"Rails rotate with banked and inverted track frame");
        auto changedGauge=style;changedGauge.components[0].offset.y=-.7;
        require(componentStyleInCoordinateUnits(changedGauge,1).railOffsets[0].lateral==-.7,
            "Engineering running references follow authored component offset edits");
        auto scaled=componentStyleInCoordinateUnits(style,2);
        require(scaled.components.front().offset.y==style.components.front().offset.y*.5,"Metre-scale visual dimensions convert to Core units");
        auto junction=style;
        auto cap=junction.components.back();cap.id="junction-cap";cap.placement=TrackComponentPlacement::Junction;
        junction.components.push_back(cap);
        require(generateRenderableTrack(straight,junction).continuousMesh.vertices.size()>mesh.continuousMesh.vertices.size(),"Explicit junction component renders at construction start");
        RegionTrackStyleOverrides legacyOverride;legacyOverride.enabled=true;legacyOverride.railRadius=.08;
        require(resolveTrackStyle(style,legacyOverride).components.size()==style.components.size(),"Legacy Region dimensions retained without blocking new topology");
        auto shifted=style;shifted.components.front().offset.x=1;
        rejects([&]{validateTrackStyle(shifted);},"Continuous forward offset is explicitly rejected");
        auto malformed=style;malformed.components.front().dimensions.z=-1;
        rejects([&]{validateTrackStyle(malformed);},"Invalid component dimensions reject before publication");
    }
    void extensionAndIntervals()
    {
        TrackFamilyRegistry registry;
        const auto source=Json::parse(serializeTrackComponents(resolveTrackConfiguration("monorail-100")));
        const Json extension={{"schemaVersion",1},{"types",Json::array({{{"id","test-100"},{"displayName","Additional family"},{"familyId","test"},{"category","Additional"},{"construction",source}}})}};
        registry.loadJson(extension.dump());require(registry.find("test-100")!=nullptr,"Additional family registers without dispatch changes");
        auto broken=extension;broken["types"][0]["id"]="test-200";broken["types"][0]["baseId"]="missing";
        rejects([&]{registry.loadJson(broken.dump());},"Unknown inheritance rejected");
        require(registry.types().size()==1,"Failed catalog registration is atomic");
        auto track=fixture("hybrid-100");auto before=track;
        auto style=track.trackStyle();style.constructionIntervals={{1,13.4,31.7,style.typeId,"topper",{}},{2,31.7,50,style.typeId,"i-box",{}}};style.nextConstructionIntervalId=3;
        track.setTrackStyle(style);
        auto mesh=generateRenderableTrack(samples(),style);geometryValid(mesh);
        require(mesh.materials.size()>=3,"Hybrid intervals have steel/rail/timber materials");
        bool boundary=false;for(const auto& v:mesh.continuousMesh.vertices) boundary=boundary || std::abs(v.position.x-13.4f)<1e-5f;
        require(boundary,"Exact interval boundary is inserted between solved samples");
        auto bad=style;bad.constructionIntervals[1].start=20;
        rejects([&]{validateTrackStyle(bad);},"Overlapping intervals rejected");
        DocumentHistory history;history.reset(before);
        auto impact=classifyDocumentTrackStyleEdit(before,track);
        require(!impact.invalidatesCanonicalTrack() && !impact.invalidatesSimulationPreview(),"Visual family changes preserve centerline and physics caches");
        history.record(track,false,impact);
        require(history.undo()->trackStyle().constructionIntervals.empty(),"Interval Undo");
        require(history.redo()->trackStyle().constructionIntervals.size()==2,"Interval Redo");
        auto json=serializeCoasterDocument(track);auto restored=deserializeCoasterDocument(json);
        const auto beforeJson=Json::parse(serializeCoasterDocument(before)),afterJson=Json::parse(json);
        for(const auto* field:{"physicalSettings","coasterSetup","trainConfiguration"})
            if(beforeJson.contains(field)) require(beforeJson.at(field)==afterJson.at(field),"Visual intervals preserve vehicle and physics authored data");
        require(restored && serializeCoasterDocument(*restored)==json,"Intervals survive save/reopen");
        const auto canonical=integrateAuthoredTrackKinematics(before,.5);const auto changed=integrateAuthoredTrackKinematics(track,.5);
        require(canonical.size()==changed.size(),"Visual intervals preserve solved sample count");
        for(std::size_t i=0;i<canonical.size();++i) require(canonical[i].position==changed[i].position,"Visual intervals preserve canonical geometry exactly");
        const auto visualization=createCenterlineVisualization(before);
        auto candidate=createTrackStylePresentationCandidate(track,visualization,impact);
        require(candidate.continuousMesh && !candidate.continuousMesh->vertices.empty(),"Native presentation-only interval regeneration");
        auto missing=Json::parse(json);missing["trackConfigurationId"]="unavailable-100";missing["trackStyle"]["construction"]["typeId"]="unavailable-100";
        for(auto& i:missing["trackStyle"]["construction"]["intervals"]) i["typeId"]="unavailable-100";
        auto missingTrack=deserializeCoasterDocument(missing.dump());
        require(missingTrack && missingTrack->trackStyle().components.size()==style.components.size(),"Unavailable catalog preserves component snapshots");
        auto shorter=track;setSectionLength(shorter.section(0),20);
        require(shorter.trackStyle().constructionIntervals[0].start==13.4,"Shaping retains authored absolute intervals");
    }
    void assetsAndSupports(const std::filesystem::path& root)
    {
        renderer::configureTrackComponentAssets(root);
        const auto imported=renderer::loadStaticMeshGlb("assets://track/hierarchy.glb",root/"tests/fixtures/track-families/hierarchy.glb");
        require(imported.vertices.size()==48 && imported.submeshes.size()==2,"GLB scene hierarchy imports both mesh occurrences");
        require(std::abs(imported.minimumPosition.x-2)<1e-6 && std::abs(imported.maximumPosition.x-4)<1e-6,"Parent and child GLB transforms compose");
        require(std::abs(imported.minimumPosition.y+4.1f)<1e-5 && std::abs(imported.maximumPosition.z-3.06f)<1e-5,"GLB axes convert after node transforms");
        require(imported.vertices[0].uv==glm::vec2{0,0} && imported.vertices[1].uv==glm::vec2{1,0}
            && imported.vertices[2].uv==glm::vec2{1,1} && imported.vertices[3].uv==glm::vec2{0,1},"GLB UV stride and metadata retained");
        auto style=resolveTrackConfiguration("truss-100");
        auto plain=generateRenderableTrack(samples(),style);
        style.components[2].asset.path="assets://track/components/example-spine.glb";
        auto custom=generateRenderableTrack(samples(),style);geometryValid(custom);
        require(custom.continuousMesh.vertices.size()!=plain.continuousMesh.vertices.size(),"Custom continuous GLB replaces one component");
        require(renderer::trackComponentAssetStatus(style.components[2].asset.path)->state==renderer::HardwareAssetLoadState::Loaded,"Loaded component diagnostic");
        auto badTile=style;badTile.components[2].assetLength=.5;
        auto boundedFallback=generateRenderableTrack(samples(),badTile);
        require(boundedFallback.continuousMesh.vertices.size()==plain.continuousMesh.vertices.size(),"Invalid custom tile bounds fall back without rejecting the document");
        require(renderer::trackComponentAssetStatus(badTile.components[2].asset.path)->usingDiagnosticFallback,"Invalid tile bounds are diagnosed");
        static_cast<void>(generateRenderableTrack(samples(),style));
        require(!renderer::trackComponentAssetStatus(style.components[2].asset.path)->usingDiagnosticFallback,"Correct tile length recovers cached geometry");
        style.components.back().asset.path="assets://track/components/example-tie.glb";
        auto repeated=generateRenderableTrack(samples(),style);
        require(!repeated.hardwareBatches.empty() && repeated.hardwareBatches[0].instances.size()>20,"Custom rigid component uses repeated GPU instances");
        style.components[2].asset.path="assets://track/components/missing.glb";
        auto fallback=generateRenderableTrack(samples(),style);
        require(renderer::trackComponentAssetStatus(style.components[2].asset.path)->usingDiagnosticFallback,"Missing custom GLB uses procedural fallback");
        require(style.components[2].asset.path.ends_with("missing.glb"),"Missing reference retained");
        // A missing reference can recover in-place after Blender exports the
        // file. Use a fresh temporary package, never a user-owned asset path.
        const auto temporary=std::filesystem::temp_directory_path()/
            ("QuantumTrackRecovery-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        require(std::filesystem::create_directory(temporary),"Fresh recovery package");
        auto recovery=resolveTrackConfiguration("truss-100");
        recovery.components[2].asset.path="assets://track/components/recovery.glb";
        renderer::configureTrackComponentAssets(temporary);
        auto absent=generateRenderableTrack(samples(),recovery);
        require(renderer::trackComponentAssetStatus(recovery.components[2].asset.path)->usingDiagnosticFallback,"Missing asset cached non-destructively");
        const auto recoveryDirectory=temporary/"assets/track/components";
        std::filesystem::create_directories(recoveryDirectory);
        std::filesystem::copy_file(root/"assets/track/components/example-spine.glb",recoveryDirectory/"recovery.glb");
        renderer::reloadTrackComponentAsset(recovery.components[2].asset.path);
        auto recovered=generateRenderableTrack(samples(),recovery);
        require(recovered.continuousMesh.vertices.size()!=absent.continuousMesh.vertices.size(),"Reload recovers a missing component without changing its reference");
        renderer::configureTrackComponentAssets(root);
        std::filesystem::remove(recoveryDirectory/"recovery.glb");
        std::filesystem::remove(recoveryDirectory);std::filesystem::remove(temporary/"assets/track");
        std::filesystem::remove(temporary/"assets");std::filesystem::remove(temporary);
        auto track=fixture("wood-100");
        SupportFrameRecipe recipe;recipe.station=10;recipe.foundationElevation=0;
        auto id=generateSupportFrame(track,SupportStyleFamily::TubularSteel,recipe);
        const auto& frame=track.supports().structures.front();
        require(std::abs(frame.nodes[2].trackAttachment->verticalOffset-(recipe.verticalOffset-.44))<1e-12,"Generated frame uses family attachment offset");
        auto edited=track.supports();edited.structures.front().members.front().profile.outerDimensions=glm::dvec2{.7};track.setSupports(edited);
        static_cast<void>(generateSupportFrame(track,SupportStyleFamily::TubularSteel,recipe,id));
        require(track.supports().structures.front().members.front().profile.outerDimensions.x==.7,"Regeneration preserves manually modified generated profile");
        WoodenSupportRunRecipe timber;timber.endStation=30;timber.foundationElevation=0;
        static_cast<void>(generateWoodenSupportRun(track,timber));
        bool attached=false;for(const auto& structure:track.supports().structures) for(const auto& node:structure.nodes)
            attached=attached || (node.trackAttachment && std::abs(node.trackAttachment->verticalOffset-(timber.attachmentVerticalOffset-.44))<1e-12);
        require(attached,"Timber generator uses family bearing offset");
    }
    void writeFixtures(const std::filesystem::path& directory)
    {
        std::filesystem::create_directories(directory);
        const auto cameraTrack=fixture("truss-100");
        const auto cameraStates=integrateAuthoredTrackKinematics(cameraTrack, .25);
        Json cameras=Json::object();
        for(double station:{70.0,90.0,165.0})
        {
            const auto sample=std::lower_bound(cameraStates.begin(),cameraStates.end(),station,
                [](const auto& state,double target){return state.distance<target;});
            require(sample!=cameraStates.end(),"Verification camera station exists");
            cameras[std::to_string(static_cast<int>(station))]={sample->position.x,sample->position.y,sample->position.z};
        }
        std::ofstream(directory/"camera-stations.json")<<cameras.dump(2);

        for(const auto& [id,def]:trackFamilyRegistry().types())
        {
            if(def.reserved || id==modernSteelTrackConfigurationId) continue;
            auto track=fixture(id);
            if(id=="hybrid-100")
            {
                auto style=track.trackStyle();style.constructionIntervals={{1,70,115,id,"topper",{}}};style.nextConstructionIntervalId=2;track.setTrackStyle(style);
            }
            std::ofstream(directory/(id+".quantum"))<<serializeCoasterDocument(track);
        }
        auto mixed=fixture("truss-200");mixed.section(1)=createRateProfileSection(45);convertSectionToPlanarArc(mixed.section(1));
        std::ofstream(directory/"mixed-regions.quantum")<<serializeCoasterDocument(mixed);
        auto topper=fixture("hybrid-100");auto topperStyle=topper.trackStyle();
        topperStyle.variantId="topper";topperStyle.components=topperStyle.variants.back().components;
        topperStyle.supportAttachment=topperStyle.variants.back().supportAttachment;
        topper.setTrackStyle(topperStyle);std::ofstream(directory/"hybrid-topper.quantum")<<serializeCoasterDocument(topper);
        auto longTrack=createNewDocument();longTrack.section(0)=createRateProfileSection(2000);longTrack.applyTrackConfiguration("truss-100");
        longTrack.setStartPose({{0,0,12},{1,0,0,0}});std::ofstream(directory/"long-layout.quantum")<<serializeCoasterDocument(longTrack);
        auto custom=fixture("truss-100");auto style=custom.trackStyle();
        style.components[2].asset.path="assets://track/components/example-spine.glb";
        style.components.back().asset.path="assets://track/components/example-tie.glb";
        custom.setTrackStyle(style);std::ofstream(directory/"custom-components.quantum")<<serializeCoasterDocument(custom);
        auto supported=fixture("wood-200");WoodenSupportRunRecipe recipe;recipe.endStation=70;recipe.foundationElevation=0;recipe.attachmentVerticalOffset=0;
        static_cast<void>(generateWoodenSupportRun(supported,recipe));std::ofstream(directory/"wood-supports.quantum")<<serializeCoasterDocument(supported);
    }
}
int main(int argc,char** argv)
{
    try
    {
        if(argc<2) throw std::invalid_argument("Supply repository root.");
        const std::filesystem::path root=argv[1];
        catalogAndGeometry();extensionAndIntervals();assetsAndSupports(root);
        if(argc==4 && std::string_view{argv[2]}=="--write-fixtures") writeFixtures(argv[3]);
        const auto begin=std::chrono::steady_clock::now();auto mesh=generateRenderableTrack(samples(2000),resolveTrackConfiguration("truss-100"));
        std::cout<<checks<<" checks passed; 2 km Truss-100 generation "<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count()<<" ms; "<<mesh.continuousMesh.vertices.size()<<" vertices, "<<mesh.continuousMesh.submeshes.size()<<" draws\n";
        return 0;
    }
    catch(const std::exception& error) { std::cerr<<"Track family failure: "<<error.what()<<'\n';return 1; }
}
