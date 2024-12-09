#pragma once

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <unordered_map>

namespace cev_localization {
    namespace config_parser {
        struct Sensor {
            std::string type;
            std::string topic;
            std::string frame_id;
            std::vector<bool> state;
            double covariance_multiplier;
            std::vector<std::string> estimator_models;
        };

        struct UpdateModel {
            std::string type;
            std::vector<std::string> parameters;
        };

        struct Config {
            // General settings
            double time_step;
            std::string odometry_topic;
            std::string main_model;

            // Sensors
            std::unordered_map<std::string, Sensor> sensors;

            // Update Models
            std::unordered_map<std::string, UpdateModel> update_models;
        };

        class ConfigParser {
        public:
            static Config loadConfig(const std::string& filePath);

        private:
            static Sensor parseSensor(const YAML::Node& sensorNode);
            static UpdateModel parseUpdateModel(const YAML::Node& modelNode);
        };
    }
}