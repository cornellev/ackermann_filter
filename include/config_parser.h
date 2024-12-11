#pragma once

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>
#include <optional>

#include "estimator.h"

namespace cev_localization {
    namespace config_parser {
        struct Sensor {
            std::string type;
            std::string topic;
            std::optional<std::string> frame_id;
            std::vector<std::string> state_mask;
            double covariance_multiplier;
            std::vector<std::string> estimator_models;
        };

        struct UpdateModel {
            std::string type;
            std::vector<std::string> state_mask;
            std::vector<std::string> estimator_models;
        };

        struct Config {
            // General settings
            double time_step;  // seconds

            std::string main_model;

            // Odometry settings
            std::string odometry_topic;
            std::string base_link_frame;
            std::string odom_frame;

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