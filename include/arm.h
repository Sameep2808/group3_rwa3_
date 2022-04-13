#ifndef __ARM_H__
#define __ARM_H__

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
// standard library
#include <string>
#include <vector>
#include <array>
#include <cstdarg>
// nist
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>
#include <nist_gear/LogicalCameraImage.h>
// custom
#include "utils.h"

namespace motioncontrol {


    /**
     * @brief class for the Gantry robot
     *
     */
    class Arm {
        public:
        /**
        * @brief Struct for preset locations
        * @todo Add new preset locations here if needed
        */
        typedef struct ArmPresetLocation {
            std::vector<double> arm_preset;  //9 joints
            std::string name;
        } start, bin, agv, grasp;

        /**
        * @brief A queued part to finalize before shipping.
        */
        struct QueuedPartToFinalize {
            const int type;
            const std::string agv;
            const geometry_msgs::Pose init_pose_in_world;
            const geometry_msgs::Pose goal_in_tray_frame;
            const std::string part_type;
            const geometry_msgs::Pose part_pose_in_frame;

            QueuedPartToFinalize(const std::string& agv_,
                                 const geometry_msgs::Pose& init_pose_in_world_,
                                 const geometry_msgs::Pose& goal_in_tray_frame_) :
                type(1),
                agv(agv_),
                init_pose_in_world(init_pose_in_world_),
                goal_in_tray_frame(goal_in_tray_frame_)
            {}
            QueuedPartToFinalize(const std::string& agv_,
                                 const std::string& part_type_,
                                 const geometry_msgs::Pose& part_pose_in_frame_) :
                type(2),
                agv(agv_),
                part_type(part_type_),
                part_pose_in_frame(part_pose_in_frame_)
            {}
        };

        Arm(ros::NodeHandle& node_handle);
        /**
         * @brief Initialize the object
         */
        void init();
        bool pickPart(std::string part_type, geometry_msgs::Pose part_pose);
        bool placePart(geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_goal_pose, std::string agv, bool* blackout_active=0);
        void testPreset(const std::vector<ArmPresetLocation>& preset_list);
        void movePart(std::string part_type, std::string camera_frame, geometry_msgs::Pose goal_in_tray_frame, std::string agv, bool* blackout_active);
        void activateGripper();
        void deactivateGripper();
        bool hasQueuedParts();
        void finalizeQueuedParts();

        void check_part_pose(geometry_msgs::Pose target_pose_in_world,std::string agv);

        void qualityControl1Callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
        void qualityControl2Callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
        void qualityControl3Callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
        void qualityControl4Callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
                
         bool& get_quality_camera1_data(){
            return quality_camera_1;
        }
          bool& get_quality_camera2_data(){
            return quality_camera_2;
        }
          bool& get_quality_camera3_data(){
            return quality_camera_3;
        }
          bool& get_quality_camera4_data(){
            return quality_camera_4;
        }

        /**
         * @brief Move the joint linear_arm_actuator_joint only
         *
         * The joint position for this joint corresponds to the y value
         * in the world frame. For instance, a value of 0 for this joint
         * moves the base of the robot to y = 0.
         *
         * @param location A preset location
         */
        void moveBaseTo(double linear_arm_actuator_joint_position);
        nist_gear::VacuumGripperState getGripperState();

        

        // Send command message to robot controller
        bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
        void goToPresetLocation(std::string location_name);

        //--preset locations;
        start home1_, home2_;
        agv agv1_, agv2_, agv3_, agv4_;

        int* counter;
        bool quality_camera_1;
        bool quality_camera_2;
        bool quality_camera_3;
        bool quality_camera_4;
        bool quality_camera[4];

        std::string get_camera_frame_of_moving_object(){
            return camera_frame_of_moving_object;
        } 

        std::string camera_frame_of_moving_object;

        private:
        std::vector<double> joint_group_positions_;
        std::vector<double> joint_arm_positions_;
        ros::NodeHandle node_;
        std::string planning_group_;
        moveit::planning_interface::MoveGroupInterface::Options arm_options_;
        moveit::planning_interface::MoveGroupInterface arm_group_;
        sensor_msgs::JointState current_joint_states_;
        control_msgs::JointTrajectoryControllerState arm_controller_state_;

        nist_gear::VacuumGripperState gripper_state_;
        // gripper state subscriber
        ros::Subscriber gripper_state_subscriber_;
        // service client
        ros::ServiceClient gripper_control_client_;
        // publishers
        ros::Publisher arm_joint_trajectory_publisher_;
        // joint states subscribers
        ros::Subscriber arm_joint_states_subscriber_;
        // controller state subscribers
        ros::Subscriber arm_controller_state_subscriber_;

        std::string part_type_name;

        // A list of parts that are queued for finalization, and a set of names
        // of AGV's that have a queued part on it
        std::vector<QueuedPartToFinalize> queued_parts;

        int *get_counter(){
            return counter;
        }
        auto get_part_type_name(){
            return part_type_name;
        }

        // callbacks
        void gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg);
        void arm_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
        void arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

        void finalizePlacedPart(const std::string& agv,
                                const std::string& part_type,
                                const geometry_msgs::Pose& part_pose_in_frame);

        void check_faulty_part(std::string part_type,geometry_msgs::Pose part_pose_in_frame,std::string agv);
        
        ros::Subscriber Arm_quality_control_sensor_1_subscriber;
        /*!< subscriber to the topic /ariac/quality_control_sensor_2 */
        ros::Subscriber Arm_quality_control_sensor_2_subscriber;
        ros::Subscriber Arm_quality_control_sensor_3_subscriber;
        ros::Subscriber Arm_quality_control_sensor_4_subscriber;
    
    };
}//namespace
#endif
