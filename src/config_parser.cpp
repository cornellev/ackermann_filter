#include "config_parser.h"

using namespace cev_localization::config_parser;

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

Sensor ConfigParser::parseSensor(const YAML::Node& sensorNode) {
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
