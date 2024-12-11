#include "config_parser.h"

using namespace cev_localization::config_parser;

Config ConfigParser::loadConfig(const std::string& filePath) {
    YAML::Node configNode = YAML::LoadFile(filePath);

    Config config;

    // Publish rate
    try {
        double rate = configNode["odometry_settings"]["publish_rate"].as<double>();
        if (rate <= 0) {
            throw std::runtime_error("Parameter `odometry_settings/publish_rate` must be greater than 0");
        }
        config.time_step = 1.0 / rate;
    } catch (YAML::Exception& e) {
        config.time_step = 0.01;
    }

    // Odometry settings
    try {
        config.base_link_frame = configNode["odometry_settings"]["base_link_frame"].as<std::string>();
    } catch (YAML::Exception& e) {
        config.base_link_frame = "base_link";
    }

    try {
        config.odom_frame = configNode["odometry_settings"]["odom_frame"].as<std::string>();
    } catch (YAML::Exception& e) {
        config.odom_frame = "odom";
    }

    try {
        config.odometry_topic = configNode["odometry_settings"]["topic"].as<std::string>();
    } catch (YAML::Exception& e) {
        config.odometry_topic = "odom";
    }
    
    // Main model
    try {
        config.main_model = configNode["odometry_settings"]["main_model"].as<std::string>();
    } catch (YAML::Exception& e) {
        throw std::runtime_error("Parameter `odometry_settings/main_model` not defined");
    }

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
    try {
        sensor.type = sensorNode["type"].as<std::string>();
    } catch (YAML::Exception& e) {
        throw std::runtime_error("Sensor type not defined");
    }
    
    try {
        sensor.topic = sensorNode["topic"].as<std::string>();
    } catch (YAML::Exception& e) {
        throw std::runtime_error("Sensor topic not defined");
    }

    try {
        sensor.frame_id = sensorNode["frame_id"].as<std::string>();
    } catch (YAML::Exception& e) {
        sensor.frame_id = std::nullopt;
    }

    // Parse state mask
    for (const auto& val: sensorNode["state"]) {
        sensor.state_mask.push_back(val.as<std::string>());
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

    try {
        model.type = modelNode["type"].as<std::string>();
    } catch (YAML::Exception& e) {
        throw std::runtime_error("Model type not defined");
    }

    // Parse state mask
    for (const auto& val: modelNode["state"]) {
        model.state_mask.push_back(val.as<std::string>());
    }

    // Parse estimator_models
    for (const auto& mod: modelNode["estimator_models"]) {
        model.estimator_models.push_back(mod.as<std::string>());
    }

    return model;
}