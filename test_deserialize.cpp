#include <quantum/coaster/CoasterDocument.hpp>
#include <iostream>
#include <string>

int main() {
    const std::string legacyJson = R"({
        "formatVersion": 1,
        "layoutMode": "Shuttle",
        "startPose": {"position": {"x": 0, "y": 0, "z": 0}, "orientation": {"w": 1, "x": 0, "y": 0, "z": 0}},
        "physicalSettings": {"initialSpeed": 0, "metersPerCoordinateUnit": 1, "gravityAcceleration": 9.81},
        "trackStyle": {"name": "Test", "geometryFamily": "DualRailTubular", "visible": true, "railsVisible": true, "railCount": 2, "railOffsets": [{"lateral": -1, "vertical": 0}, {"lateral": 1, "vertical": 0}], "railRadius": 0.1, "railRadialSegments": 8, "railMaterial": {"baseColor": {"r": 1, "g": 1, "b": 1, "a": 1}, "metallicFactor": 1, "roughnessFactor": 0.5}, "spine": {"enabled": true, "type": "Tubular", "offset": {"lateral": 0, "vertical": 0}, "dimensions": {"x": 0.2, "y": 0.2}, "radialSegments": 8, "material": {"baseColor": {"r": 0.5, "g": 0.5, "b": 0.5, "a": 1}, "metallicFactor": 1, "roughnessFactor": 0.5}}, "repeatingHardware": []},
        "coasterSetup": {"styleId": "modern_steel", "carsPerTrain": 2, "options": [], "heartline": {"enabled": false, "offsetMeters": 0}},
        "supports": {"nextStructureId": 1, "structures": []},
        "trackDevices": {"nextId": 2, "devices": [{"id": 1, "kind": "Launch", "name": "Launch", "enabled": true, "startStationMeters": 0, "endStationMeters": 10, "targetAccelerationMetersPerSecondSquared": 3, "maximumForceNewtons": 5000}]},
        "sections": [{"kind": "RateProfiles", "length": 100, "rateProfiles": {"pitch": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0, "domainEnd": 100, "valueBegin": 0, "valueEnd": 0, "type": "Linear"}}]}, "yaw": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0, "domainEnd": 100, "valueBegin": 0, "valueEnd": 0, "type": "Linear"}}]}, "roll": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0, "domainEnd": 100, "valueBegin": 0, "valueEnd": 0, "type": "Linear"}}]}}]
    })";

    auto result = quantum::coaster::deserializeCoasterDocument(legacyJson);
    if (result.has_value()) {
        std::cout << "Success!" << std::endl;
        return 0;
    } else {
        std::cerr << "Error: " << result.error() << std::endl;
        return 1;
    }
}