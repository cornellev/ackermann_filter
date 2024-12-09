#pragma once

#include "sensor.h"

namespace cev_localization {

    template<typename T>
    class RosSensor : public ckf::Sensor {
    protected:
        ckf::M multiplier = ckf::M::Zero();

        void new_time_handler(double time) {
            previous_update_time = most_recent_update_time;
            most_recent_update_time = time;
        }

    public:
        RosSensor(ckf::V state, ckf::M covariance,
            std::vector<std::shared_ptr<ckf::Model>> dependents)
            : Sensor(state, covariance, dependents) {}

        /**
         * Handle a new message. Meant to be used as or in a message subscriber.
         */
        void msg_handler(typename T::SharedPtr msg) {
            ckf::StatePackage update = msg_update(msg);
            updateInternals(update);
            update_dependents();
        }

        /**
         * Update the state with a new message, and return the time of the new message
         */
        virtual ckf::StatePackage msg_update(typename T::SharedPtr msg) = 0;

        ckf::M state_matrix_multiplier() {
            return multiplier;
        }
    };
}