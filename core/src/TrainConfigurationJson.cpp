#include <quantum/coaster/TrainConfiguration.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace quantum::coaster
{
    namespace
    {
        using nlohmann::json;
        json vector(const glm::dvec3& v) { return {v.x, v.y, v.z}; }
        glm::dvec3 vector(const json& value)
        {
            if (!value.is_array() || value.size() != 3) throw std::invalid_argument("Train vector requires three numbers.");
            glm::dvec3 result;
            for (int i = 0; i < 3; ++i)
            {
                if (!value[i].is_number()) throw std::invalid_argument("Train vector requires finite numbers.");
                result[i] = value[i].get<double>();
                if (!std::isfinite(result[i])) throw std::invalid_argument("Train vector requires finite numbers.");
            }
            return result;
        }
        void fields(const json& value, std::initializer_list<std::string_view> names)
        {
            if (!value.is_object()) throw std::invalid_argument("Train data must be an object.");
            for (const auto& [key, item] : value.items())
                if (std::ranges::find(names, key) == names.end()) throw std::invalid_argument("Unknown train field: " + key);
            for (auto key : names)
                if (!value.contains(key)) throw std::invalid_argument("Missing train field: " + std::string(key));
        }
        std::uint64_t id(const json& value)
        {
            if (!value.is_number_unsigned() && (!value.is_number_integer() || value.get<std::int64_t>() < 0))
                throw std::invalid_argument("Train IDs require nonnegative integers.");
            return value.get<std::uint64_t>();
        }
        double number(const json& value)
        {
            if (!value.is_number()) throw std::invalid_argument("Train physical value requires a number.");
            const auto result = value.get<double>();
            if (!std::isfinite(result)) throw std::invalid_argument("Train physical value must be finite.");
            return result;
        }
        json physical(const physics::TrainCarDefinition& p)
        {
            const auto& c = p.car;
            json bogies = json::array();
            for (const auto& b : c.bogies)
            {
                json contacts = json::array();
                for (const auto& contact : b.contacts)
                    contacts.push_back({{"role", static_cast<unsigned>(contact.role)},
                        {"positionMeters", vector(contact.localPositionMeters)}, {"normal", vector(contact.contactNormalLocal)},
                        {"clearanceMeters", contact.clearanceMeters}});
                bogies.push_back({{"positionMeters", vector(b.referencePositionMeters)}, {"contacts", contacts}});
            }
            json inertia = json::array();
            for (int row = 0; row < 3; ++row)
                inertia.push_back({c.dryInertiaTensorBodyKgM2[0][row], c.dryInertiaTensorBodyKgM2[1][row], c.dryInertiaTensorBodyKgM2[2][row]});
            return {{"dryMassKilograms", c.dryMassKilograms}, {"dryCogMeters", vector(c.dryCenterOfGravityMeters)},
                {"bodyDimensionsMeters", vector(c.bodyDimensionsMeters)}, {"inertiaKgM2", inertia},
                {"frontHitchMeters", vector(c.frontHitchPositionMeters)}, {"rearHitchMeters", vector(c.rearHitchPositionMeters)},
                {"bogies", bogies}, {"loadMassKilograms", p.loadout.massKilograms}, {"loadCogMeters", vector(p.loadout.centerOfMassMeters)},
                {"dragAreaSquareMeters", c.aerodynamicDragAreaSquareMeters}, {"aerodynamicCenterMeters", vector(c.aerodynamicCenterLocalMeters)}};
        }
        physics::TrainCarDefinition physical(const json& j)
        {
            fields(j, {"dryMassKilograms", "dryCogMeters", "bodyDimensionsMeters", "inertiaKgM2", "frontHitchMeters",
                "rearHitchMeters", "bogies", "loadMassKilograms", "loadCogMeters", "dragAreaSquareMeters", "aerodynamicCenterMeters"});
            physics::TrainCarDefinition p;
            auto& c = p.car;
            c.dryMassKilograms = number(j.at("dryMassKilograms"));
            c.dryCenterOfGravityMeters = vector(j.at("dryCogMeters"));
            c.bodyDimensionsMeters = vector(j.at("bodyDimensionsMeters"));
            const auto& inertia = j.at("inertiaKgM2");
            if (!inertia.is_array() || inertia.size() != 3) throw std::invalid_argument("Inertia requires three matrix rows.");
            for (int row = 0; row < 3; ++row)
            {
                const auto values = vector(inertia[row]);
                for (int column = 0; column < 3; ++column) c.dryInertiaTensorBodyKgM2[column][row] = values[column];
            }
            c.frontHitchPositionMeters = vector(j.at("frontHitchMeters"));
            c.rearHitchPositionMeters = vector(j.at("rearHitchMeters"));
            if (!j.at("bogies").is_array()) throw std::invalid_argument("Bogies require an array.");
            for (const auto& b : j.at("bogies"))
            {
                fields(b, {"positionMeters", "contacts"});
                physics::BogieDefinition bogie{vector(b.at("positionMeters"))};
                if (!b.at("contacts").is_array()) throw std::invalid_argument("Contacts require an array.");
                for (const auto& contact : b.at("contacts"))
                {
                    fields(contact, {"role", "positionMeters", "normal", "clearanceMeters"});
                    const auto role = id(contact.at("role"));
                    if (role > 2) throw std::invalid_argument("Unknown bogie contact role.");
                    bogie.contacts.push_back({static_cast<physics::BogieContactRole>(role), vector(contact.at("positionMeters")),
                        vector(contact.at("normal")), number(contact.at("clearanceMeters"))});
                }
                c.bogies.push_back(std::move(bogie));
            }
            p.loadout.massKilograms = number(j.at("loadMassKilograms"));
            p.loadout.centerOfMassMeters = vector(j.at("loadCogMeters"));
            c.aerodynamicDragAreaSquareMeters = number(j.at("dragAreaSquareMeters"));
            c.aerodynamicCenterLocalMeters = vector(j.at("aerodynamicCenterMeters"));
            return p;
        }
    }

    std::string serializeTrainConfiguration(const TrainConfiguration& c)
    {
        validateTrainConfiguration(c);
        json cars = json::array();
        for (const auto& car : c.cars)
            cars.push_back({{"id", car.id}, {"name", car.name}, {"role", trainCarRoleName(car.role)},
                {"physical", physical(car.physical)}, {"connectorLengthMeters", car.rearConnection.rigidLengthMeters},
                {"visualAsset", car.visualAsset}, {"visualTranslationMeters", vector(car.visualAlignment.translationMeters)},
                {"visualRotationDegrees", vector(car.visualAlignment.rotationDegrees)}});
        const auto& r = c.resistance;
        return json{{"schemaVersion", 2}, {"trainId", c.trainId}, {"name", c.name}, {"nextCarId", c.nextCarId}, {"cars", cars},
            {"defaultPhysical", physical(c.repeatedCar)}, {"defaultConnectorLengthMeters", c.repeatedConnection.rigidLengthMeters},
            {"resistance", {{"mechanicalForceNewtons", r.constantMechanicalForceNewtons},
                {"linearCoefficient", r.linearResistanceCoefficientNewtonSecondsPerMeter},
                {"airDensity", r.airDensityKilogramsPerCubicMeter}, {"dragAreaSquareMeters", r.dragAreaSquareMeters},
                {"rollingCoefficient", r.rollingResistanceCoefficient}}}}.dump();
    }

    TrainConfiguration deserializeTrainConfiguration(const std::string& text, bool validate)
    {
        try
        {
            const auto j = json::parse(text);
            fields(j, {"schemaVersion", "trainId", "name", "nextCarId", "cars", "defaultPhysical", "defaultConnectorLengthMeters", "resistance"});
            if (id(j.at("schemaVersion")) != 2) throw std::invalid_argument("Unsupported train schemaVersion.");
            TrainConfiguration c;
            c.trainId = id(j.at("trainId")); c.nextCarId = id(j.at("nextCarId")); c.name = j.at("name").get<std::string>();
            c.repeatedCar = physical(j.at("defaultPhysical"));
            c.repeatedConnection.rigidLengthMeters = number(j.at("defaultConnectorLengthMeters"));
            if (!j.at("cars").is_array() || j.at("cars").size() > 32) throw std::invalid_argument("Cars require an array of at most 32 entries.");
            for (const auto& car : j.at("cars"))
            {
                fields(car, {"id", "name", "role", "physical", "connectorLengthMeters", "visualAsset", "visualTranslationMeters", "visualRotationDegrees"});
                AuthoredTrainCar entry;
                entry.id = id(car.at("id")); entry.name = car.at("name").get<std::string>();
                entry.role = trainCarRoleFromName(car.at("role").get<std::string>());
                entry.physical = physical(car.at("physical"));
                entry.rearConnection.rigidLengthMeters = number(car.at("connectorLengthMeters"));
                entry.visualAsset = car.at("visualAsset").get<std::string>();
                entry.visualAlignment = {vector(car.at("visualTranslationMeters")), vector(car.at("visualRotationDegrees"))};
                c.cars.push_back(std::move(entry));
            }
            c.carCount = c.cars.size();
            const auto& r = j.at("resistance");
            fields(r, {"mechanicalForceNewtons", "linearCoefficient", "airDensity", "dragAreaSquareMeters", "rollingCoefficient"});
            c.resistance = {number(r.at("mechanicalForceNewtons")), number(r.at("linearCoefficient")),
                number(r.at("airDensity")), number(r.at("dragAreaSquareMeters")), number(r.at("rollingCoefficient"))};
            if (validate) validateTrainConfiguration(c);
            return c;
        }
        catch (const json::exception& error) { throw std::invalid_argument("Train JSON: " + std::string(error.what())); }
    }
}
