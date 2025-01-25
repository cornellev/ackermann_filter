#include "std_ros_sensors.h"
#include <cmath>

using namespace cev_localization::standard_ros_sensors;

/* IMU SENSOR */

double IMUSensor::pos_mod(double angle) {
    return fmod(fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
}

IMUSensor::IMUSensor(std::string topic, V state, M covariance,
    std::vector<std::shared_ptr<Model>> dependents, std::vector<std::string> state_mask,
    bool use_message_covariance, bool relative)
    : RosSensor<sensor_msgs::msg::Imu>(topic, state, covariance, dependents),
      initialized(false),
      relative(relative),
      initial_yaw(0),
      last_reported_yaw(0),
      last_sensor_raw_yaw(0) {
    multiplier = Estimator::state_mask_to_matrix(state_mask);
    this->use_message_covariance = use_message_covariance;
}

StatePackage IMUSensor::msg_update(sensor_msgs::msg::Imu::SharedPtr msg) {
    StatePackage estimate = get_internals();
    estimate.update_time = msg->header.stamp.sec + (msg->header.stamp.nanosec / 1e9);

    // -1 means that the corresponding estimate is invalid
    if (msg->linear_acceleration_covariance[0] != -1) {
        estimate.state[ckf::state::d2_x] = msg->linear_acceleration.x;
        estimate.state[ckf::state::d2_y] = msg->linear_acceleration.y;

        if (this->use_message_covariance) {
            estimate.covariance(ckf::state::d2_x,
                ckf::state::d2_x) = msg->linear_acceleration_covariance[0];
            estimate.covariance(ckf::state::d2_x,
                ckf::state::d2_y) = msg->linear_acceleration_covariance[1];
            estimate.covariance(ckf::state::d2_y,
                ckf::state::d2_x) = msg->linear_acceleration_covariance[3];
            estimate.covariance(ckf::state::d2_y,
                ckf::state::d2_y) = msg->linear_acceleration_covariance[4];
        }
    }

    if (msg->orientation_covariance[0] != -1) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        if (relative && !initialized) {
            last_sensor_raw_yaw = yaw;
            last_reported_yaw = 0.0;
            initialized = true;

            estimate.state[ckf::state::yaw] = 0.0;
            return estimate;
        }

        double modded_yaw_diff = fmod(yaw - last_sensor_raw_yaw, 2 * M_PI);

        if (modded_yaw_diff > M_PI) {
            modded_yaw_diff -= 2 * M_PI;
        } else if (modded_yaw_diff < -M_PI) {
            modded_yaw_diff += 2 * M_PI;
        }

        last_reported_yaw += modded_yaw_diff;
        last_sensor_raw_yaw = yaw;

        estimate.state[ckf::state::yaw] = last_reported_yaw;

        if (this->use_message_covariance) {
            estimate.covariance(ckf::state::yaw, ckf::state::yaw) = msg->orientation_covariance[8];
        }
    }

    return estimate;
}

/* RAW SENSOR */

RawSensor::RawSensor(std::string topic, V state, M covariance,
    std::vector<std::shared_ptr<Model>> dependents, std::vector<std::string> state_mask,
    bool use_message_covariance)
    : RosSensor<cev_msgs::msg::SensorCollect>(topic, state, covariance, dependents) {
    multiplier = Estimator::state_mask_to_matrix(state_mask);
    this->use_message_covariance = use_message_covariance;
}

StatePackage RawSensor::msg_update(cev_msgs::msg::SensorCollect::SharedPtr msg) {
    StatePackage estimate = get_internals();
    estimate.update_time = msg->timestamp;

    estimate.state[ckf::state::d_x] = msg->velocity;
    estimate.state[ckf::state::tau] = msg->steering_angle;

    // if (this->use_message_covariance) { // TODO: Implement covariance
    //     estimate.covariance(ckf::state::d_x, ckf::state::d_x) = msg->velocity_covariance;
    //     estimate.covariance(ckf::state::tau, ckf::state::tau) = msg->steering_angle_covariance;
    // }

    return estimate;
}