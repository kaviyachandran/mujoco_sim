#include "ee_impedance_controller.h"

namespace ee_impedance_controller
{
    bool EEImpedanceController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &nh)
    {
        
        ROS_INFO("Initializing Cartesian impedance controller in namespace: %s", nh.getNamespace().c_str());

        // Fetch parameters
        nh.param<std::string>("end_effector", this->_end_effector_frame, "iiwa_link_ee");
        ROS_INFO_STREAM("End effektor link is: " << this->_end_effector_frame);
        // Frame for applying commanded Cartesian wrenches
        nh.param<std::string>("wrench_ee_frame", this->_wrench_ee_frame, this->_end_effector_frame);
       
        bool enable_trajectories{true};
        nh.param<bool>("handle_trajectories", enable_trajectories, true);
        nh.param<double>("update_frequency", this->_update_frequency, 500.);
       
        this->_root_link = "base_link"; // Get this from mujoco
        nh.setParam("root_frame", this->_root_link);

        // Initialize base_tools and member variables
        std::string robot = MjSim::robots.back();
        std::vector<std::string> joint_names = MjSim::joint_names[robot];
        this->_num_joints = joint_names.size();

        if (this->_num_joints < 6)
        {
            ROS_WARN("Number of joints is below 6. Functions might be limited.");
        }
        if (this->_num_joints < 7)
        {
            ROS_WARN("Number of joints is below 7. No redundant joint for nullspace.");
        }
        //this->tau_m_ = Eigen::VectorXd(this->n_joints_);

        // Needs to be after base_tools init since the wrench callback calls it
        // if (dynamic_reconfigure && !this->initDynamicReconfigure(nh))
        // {
        // return false;
        // }

        ROS_INFO("Finished initialization.");
        return true;
    }

}