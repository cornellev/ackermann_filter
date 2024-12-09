#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "model.h"
#include "ros_sensor.h"
#include "config_parser.h"
#include "model.h"
#include "sensor.h"
#include "standard_models.h"
#include "std_ros_sensors.h"

#include <tf2_ros/transform_broadcaster.h>

#include <iostream>

using std::placeholders::_1;

using namespace ckf;
using namespace cev_localization;

class AckermannEkfNode : public rclcpp::Node {
public:
    AckermannEkfNode(): Node("AckermannEkfNode") {
        RCLCPP_INFO(this->get_logger(), "Initializing Ackermann EKF Node");

        this->declare_parameter<std::string>("config_file", "config/ekf_real.yml");
        std::string config_file_path = this->get_parameter("config_file").as_string();

        // Load the YAML configurations
        config_parser::Config config = config_parser::ConfigParser::loadConfig(config_file_path);

        std::unordered_map<std::string, std::shared_ptr<ckf::Model>> update_models;

        for (std::pair<const std::string, config_parser::UpdateModel> model: config.update_models) {
            std::string name = model.first;
            config_parser::UpdateModel& mod = model.second;

            if (update_models.find(name) != update_models.end()) {
                RCLCPP_ERROR(this->get_logger(), "Model %s already exists", name.c_str());
                throw std::runtime_error("Model already exists");
            }

            if (mod.type == "ACKERMANN") {
                update_models[name] = std::make_shared<ckf::standard_models::AckermannModel>(
                    V::Zero(), M::Identity() * .1, M::Identity() * .1, .185);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unknown model type: %s", mod.type.c_str());
                throw std::runtime_error("Unknown model type");
            }
        }

        for (std::pair<const std::string, config_parser::Sensor> sensor: config.sensors) {
            std::string name = sensor.first;
            config_parser::Sensor& sen = sensor.second;

            if (sensors.find(name) != sensors.end()) {
                RCLCPP_ERROR(this->get_logger(), "Sensor %s already exists", name.c_str());
                throw std::runtime_error("Sensor already exists");
            }

            // Create array for models to bind to
            std::vector<std::shared_ptr<ckf::Model>> models;
            for (std::string& model_name: sen.estimator_models) {
                if (update_models.find(model_name) == update_models.end()) {
                    RCLCPP_ERROR(this->get_logger(), "Model %s does not exist", model_name.c_str());
                    throw std::runtime_error("Model does not exist");
                } else {
                    models.push_back(update_models[model_name]);
                }
            }

            if (sen.type == "IMU") {
                auto sensor = std::make_shared<standard_ros_sensors::IMUSensor>(sen.topic,
                    V::Zero(), M::Identity() * .1, models, sen.state);

                sensors[name] = sensor;

                sensor_subscribers[name] = this->create_subscription<sensor_msgs::msg::Imu>(
                    sen.topic, 10, [sensor](const sensor_msgs::msg::Imu::SharedPtr msg) {
                        sensor->msg_handler(msg);
                    });

            } else if (sen.type == "RAW") {
                auto sensor = std::make_shared<standard_ros_sensors::RawSensor>(sen.topic,
                    V::Zero(), M::Identity() * .1, models, sen.state);

                sensors[name] = sensor;

                sensor_subscribers[name] = this->create_subscription<cev_msgs::msg::SensorCollect>(
                    sen.topic, 10, [sensor](const cev_msgs::msg::SensorCollect::SharedPtr msg) {
                        sensor->msg_handler(msg);
                    });
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unknown sensor type: %s", sen.type.c_str());
                throw std::runtime_error("Unknown sensor type");
            }
        }
    }

private:
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> sensor_subscribers;
    std::unordered_map<std::string, std::shared_ptr<ckf::Model>> update_models;
    std::unordered_map<std::string, std::shared_ptr<ckf::Sensor>> sensors;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannEkfNode>());
    rclcpp::shutdown();
    return 0;
}