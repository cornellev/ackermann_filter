#include "config_parser.h"

using namespace cev_localization::config_parser;
using namespace cev_localization::standard_ros_sensors;
using namespace ckf::standard_models;

Config ConfigParser::loadConfig(const std::string& filePath) {
    YAML::Node configNode = YAML::LoadFile(filePath);

    Config config;

    // Parse general settings
    config.time_step = configNode["odometry_settings"]["time_step"].as<double>();
    config.odometry_topic = configNode["odometry_settings"]["topic"].as<std::string>();
    config.main_model = configNode["main_model"].as<std::string>();

    // Parse sensors
    for (const auto& sensorEntry: configNode["sensors"]) {
        const std::string sensorName = sensorEntry.first.as<std::string>();
        config.sensors[sensorName] = parseSensor(sensorEntry.second);
    }

    // Parse update models
    for (const auto& modelEntry: configNode["update_models"]) {
        const std::string modelName = modelEntry.first.as<std::string>();
        config.update_models[modelName] = parseUpdateModel(modelEntry.second);
    }

    return config;
}

cev_localization::config_parser::Sensor ConfigParser::parseSensor(const YAML::Node& sensorNode) {
    Sensor sensor;
    sensor.type = sensorNode["type"].as<std::string>();
    sensor.topic = sensorNode["topic"].as<std::string>();
    sensor.frame_id = sensorNode["frame_id"].as<std::string>();

    // Parse state as a vector of bools
    for (const auto& val: sensorNode["state"]) {
        sensor.state.push_back(val.as<bool>());
    }

    // Parse covariance_multiplier
    sensor.covariance_multiplier = sensorNode["covariance_multiplier"].as<double>();

    // Parse estimator_models
    for (const auto& model: sensorNode["estimator_models"]) {
        sensor.estimator_models.push_back(model.as<std::string>());
    }

    return sensor;
}

UpdateModel ConfigParser::parseUpdateModel(const YAML::Node& modelNode) {
    UpdateModel model;
    model.type = modelNode["type"].as<std::string>();
    for (const auto& param: modelNode["parameters"]) {
        model.parameters.push_back(param.as<std::string>());
    }
    return model;
}

std::pair<std::unordered_map<std::string, std::shared_ptr<ckf::Model>>,
    std::unordered_map<std::string, std::shared_ptr<ckf::Sensor>>>
ConfigParser::parseConfig(Config config) {
    std::unordered_map<std::string, std::shared_ptr<ckf::Model>> update_models;
    std::unordered_map<std::string, std::shared_ptr<ckf::Sensor>> sensors;

    for (std::pair<const std::string, config_parser::UpdateModel> model: config.update_models) {
        std::string name = model.first;
        config_parser::UpdateModel& mod = model.second;

        if (update_models.find(name) != update_models.end()) {
            throw std::runtime_error("Model already exists");
        }

        if (mod.type == "ACKERMANN") {
            update_models[name] = std::make_shared<AckermannModel>(V::Zero(), M::Identity() * .1,
                M::Identity() * .1, .185);
        } else {
            throw std::runtime_error("Unknown model type");
        }
    }

    for (std::pair<const std::string, config_parser::Sensor> sensor: config.sensors) {
        std::string name = sensor.first;
        config_parser::Sensor& sen = sensor.second;

        if (sensors.find(name) != sensors.end()) {
            throw std::runtime_error("Sensor already exists");
        }

        // Make array for models to bind to
        std::vector<std::shared_ptr<Model>> models;
        for (std::string& model_name: sen.estimator_models) {
            if (update_models.find(model_name) == update_models.end()) {
                throw std::runtime_error("Model does not exist");
            } else {
                models.push_back(update_models[model_name]);
            }
        }

        if (sen.type == "IMU") {
            sensors[name] = std::make_shared<IMUSensor>(sen.topic, V::Zero(), M::Identity() * .1,
                models, sen.state);
        } else if (sen.type == "RAW") {
            sensors[name] = std::make_shared<RawSensor>(sen.topic, V::Zero(), M::Identity() * .1,
                models, sen.state);
        } else {
            throw std::runtime_error("Unknown sensor type");
        }
    }

    return std::pair<std::unordered_map<std::string, std::shared_ptr<ckf::Model>>,
        std::unordered_map<std::string, std::shared_ptr<ckf::Sensor>>>(update_models, sensors);
}