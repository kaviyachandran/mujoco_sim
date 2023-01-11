#pragma once

#include <ros/ros.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include "mj_sim.h"

namespace ee_impedance_controller {

class EEImpedanceController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{

    public:
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &nh) override;

        void update(const ros::Time& time, const ros::Duration& period) override;

        void starting(const ros::Time& time) override;

        void stopping(const ros::Time& time) override;

    private:
        std::string _end_effector_frame; //end-effector link name
        std::string _wrench_ee_frame; // Frame for the application of the commanded wrench 
        double _update_frequency;
        std::string _root_link;
        std::size_t _num_joints;

};

PLUGINLIB_EXPORT_CLASS(ee_impedance_controller::EEImpedanceController, 
                            controller_interface::ControllerBase);

}//namespace ee_impedance_controller