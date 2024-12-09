#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "model.h"
#include "ros_sensor.h"
#include "std_ros_sensors.h"
#include "config_parser.h"

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

        // std::unordered_map<std::string, std::shared_ptr<Model>> update_models;
        // std::unordered_map<std::string, std::shared_ptr<Model>> sensors;

        // for (std::pair<const std::string, config_parser::UpdateModel> model:
        // config.update_models) {
        //     std::string name = model.first;
        //     config_parser::UpdateModel& mod = model.second;

        //     if (update_models.find(name) != update_models.end()) {
        //         RCLCPP_ERROR(this->get_logger(), "Model %s already exists", name.c_str());
        //         throw std::runtime_error("Model already exists");
        //     }

        //     if (mod.type == "ACKERMANN") {
        //         update_models[name] = std::make_shared<AckermannModel>(V::Zero(),
        //             M::Identity() * .1, M::Identity() * .1, .185);
        //     } else {
        //         RCLCPP_ERROR(this->get_logger(), "Unknown model type: %s", mod.type.c_str());
        //         throw std::runtime_error("Unknown model type");
        //     }
        // }

        // for (std::pair<const std::string, config_parser::Sensor> sensor: config.sensors) {
        //     std::string name = sensor.first;
        //     config_parser::Sensor& sen = sensor.second;

        //     if (sensors.find(name) != sensors.end()) {
        //         RCLCPP_ERROR(this->get_logger(), "Sensor %s already exists", name.c_str());
        //         throw std::runtime_error("Sensor already exists");
        //     }

        //     // Make array for models to bind to
        //     std::vector<std::shared_ptr<Model>> models;
        //     for (std::string& model_name: sen.estimator_models) {
        //         if (update_models.find(model_name) == update_models.end()) {
        //             RCLCPP_ERROR(this->get_logger(), "Model %s does not exist",
        //             model_name.c_str()); throw std::runtime_error("Model does not exist");
        //         } else {
        //             models.push_back(update_models[model_name]);
        //         }
        //     }

        //     if (sen.type == "IMU") {
        //         sensors[name] = std::make_shared<IMUSensor>(V::Zero(), M::Identity() * .1,
        //         models,
        //             sen.state);
        //     } else if (sen.type == "RAW") {
        //         sensors[name] = std::make_shared<RawSensor>(V::Zero(), M::Identity() * .1,
        //         models,
        //             sen.state);
        //     } else {
        //         RCLCPP_ERROR(this->get_logger(), "Unknown sensor type: %s", sen.type.c_str());
        //         throw std::runtime_error("Unknown sensor type");
        //     }
        // }

        // V start_state = V::Zero();

        // model->force_state(start_state);

        // imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1,
        //     std::bind(&IMUSensor::msg_handler, &imu, _1));

        // odom_sub = this->create_subscription<cev_msgs::msg::SensorCollect>("sensor_collect", 1,
        //     std::bind(&RawSensor::msg_handler, &odom, _1));

        // timer = this->create_wall_timer(std::chrono::milliseconds(10),
        //     std::bind(&AckermannEkfNode::timer_callback, this));

        // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 1);
    }

    // void timer_callback() {
    //     double time = get_clock()->now().seconds();

    //     // model->update(time);
    //     // V state = model->get_state();

    //     std::pair<V, M> prediction = model->predict(time);
    //     V state = prediction.first;
    //     M covariance = prediction.second;

    //     nav_msgs::msg::Odometry odom_msg;
    //     odom_msg.header.stamp = this->now();
    //     odom_msg.header.frame_id = "odom";
    //     odom_msg.child_frame_id = "base_link";

    //     odom_msg.pose.pose.position.x = state[x__];
    //     odom_msg.pose.pose.position.y = state[y__];
    //     odom_msg.pose.pose.position.z = 0.0;

    //     odom_msg.pose.covariance[0] = covariance(x__, x__);
    //     odom_msg.pose.covariance[7] = covariance(y__, y__);
    //     odom_msg.pose.covariance[35] = covariance(yaw__, yaw__);

    //     tf2::Quaternion q = tf2::Quaternion();
    //     q.setRPY(0, 0, state[yaw__]);
    //     q = q.normalized();
    //     odom_msg.pose.pose.orientation.x = q.x();
    //     odom_msg.pose.pose.orientation.y = q.y();
    //     odom_msg.pose.pose.orientation.z = q.z();
    //     odom_msg.pose.pose.orientation.w = q.w();

    //     odom_msg.twist.twist.linear.x = state[d_x__];
    //     odom_msg.twist.twist.linear.y = state[d_y__];
    //     odom_msg.twist.twist.angular.z = state[d_yaw__];

    //     odom_msg.twist.covariance[0] = covariance(d_x__, d_x__);
    //     odom_msg.twist.covariance[7] = covariance(d_y__, d_y__);
    //     odom_msg.twist.covariance[35] = covariance(d_yaw__, d_yaw__);

    //     odom_pub->publish(odom_msg);

    //     geometry_msgs::msg::TransformStamped transformStamped;

    //     transformStamped.header.stamp = this->now();
    //     transformStamped.header.frame_id = "odom";
    //     transformStamped.child_frame_id = "base_link";

    //     transformStamped.transform.translation.x = state[x__];
    //     transformStamped.transform.translation.y = state[y__];
    //     transformStamped.transform.translation.z = state[z__];

    //     transformStamped.transform.rotation.x = q.x();
    //     transformStamped.transform.rotation.y = q.y();
    //     transformStamped.transform.rotation.z = q.z();
    //     transformStamped.transform.rotation.w = q.w();

    //     tf_broadcaster_->sendTransform(transformStamped);
    // }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<cev_msgs::msg::SensorCollect>::SharedPtr odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Wall clock for publishing ackermann model state
    rclcpp::TimerBase::SharedPtr timer;

    // Odometry message publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    // std::shared_ptr<AckermannModel> model = std::make_shared<AckermannModel>(
    //     AckermannModel(V::Zero(), M::Identity() * .1, M::Identity() * .1, .185));

    // IMUSensor imu = IMUSensor(V::Zero(), M::Identity() * .1, {model});

    // RawSensor odom = RawSensor(V::Zero(), M::Identity() * .1, {model});
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannEkfNode>());
    rclcpp::shutdown();
    return 0;
}