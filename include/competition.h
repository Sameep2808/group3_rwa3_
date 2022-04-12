#ifndef __COMPETITION_H__
#define __COMPETITION_H__

#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include <nist_gear/Order.h>
#include <nist_gear/KittingShipment.h>
#include <nist_gear/LogicalCameraImage.h>

namespace motioncontrol {
    /**
     * @brief Competition class
     *
     */
    class Competition
    {
        public:
        /**
         * @brief Construct a new Competition object
         *
         * @param node Node handle
         */
        explicit Competition(ros::NodeHandle& node);
        /**
         * @brief Initialize components of the class (suscribers, publishers, etc)
         *
         */
        void init();
        /**
         * @brief Start the competition through ROS service
         *
         */
        void startCompetition();
        /**
         * @brief End competition through ROS service
         *
         */
        void endCompetition();
        /**
         * @brief Get the state of the competition (init, end, etc)
         *
         * @param msg
         */
        void competitionStateCallback(const std_msgs::String::ConstPtr& msg);
        /**
         * @brief Time since the competition started
         *
         * @param msg
         */
        void competitionClockCallback(const rosgraph_msgs::Clock::ConstPtr& msg);


        void competitionOrderCallback(const nist_gear::Order::ConstPtr& msg);
        void logicalCamera1Callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
        void logicalCamera2Callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
        void logicalCamera3Callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
        void logicalCamera4Callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
        /**
         * @brief Get the Clock objectGet
         *
         * @return double
         */
        double getClock();
        /**
         * @brief Get the Start Time object
         *
         * @return double
         */
        double getStartTime();
        /**
         * @brief Get the Competition State object
         *
         * @return std::string
         */
        std::string getCompetitionState();

        std::string order_id;

        std::vector<nist_gear::KittingShipment> getKittingShipments(){
            return competition_kitting_shipments_;
        }

        std::string get_order_id(){
            return order_id;
        }

        const std::map<std::string, std::vector<std::string> >& get_camera_data(){
            return logical_camera_map_;
        }

        const std::pair<std::string, std::string >& get_logical_camera1_data() {
            return logical_camera_1_;
        }

        const std::pair<std::string, std::string >& get_logical_camera2_data() {
            return logical_camera_2_;
        }

        const std::pair<std::string, std::string >& get_logical_camera3_data() {
            return logical_camera_3_;
        }

        const std::pair<std::string, std::string >& get_logical_camera4_data() {
            return logical_camera_4_;
        }

        


        private:
        /*!< node handle for this class */
        ros::NodeHandle node_;
        /*!< state of the competition */
        std::string competition_state_;
        /*!< current order */
        std::vector<nist_gear::KittingShipment> competition_kitting_shipments_;
        /*!< current score during the trial */
        double current_score_;
        /*!< clock to check if we are close to the time limit */
        ros::Time competition_clock_;
        /*!< wall time in second */
        double competition_start_time_;
        /*!< subscriber to the topic /ariac/current_score */
        ros::Subscriber current_score_subscriber_;
        /*!< subscriber to the topic /ariac/competition_state */
        ros::Subscriber competition_state_subscriber_;
        /*!< subscriber to the topic /clock */
        ros::Subscriber competition_clock_subscriber_;
        /*!< subscriber to the topic /ariac/orders */
        ros::Subscriber competition_order_subscriber_;
        /*!< subscriber to the topic /ariac/logical_camera_1 */
        ros::Subscriber competition_logical_camera_1_subscriber_;
        /*!< subscriber to the topic /ariac/logical_camera_2 */
        ros::Subscriber competition_logical_camera_2_subscriber_;
        /*!< subscriber to the topic /ariac/logical_camera_3 */
        ros::Subscriber competition_logical_camera_3_subscriber_;
        /*!< subscriber to the topic /ariac/logical_camera_4 */
        ros::Subscriber competition_logical_camera_4_subscriber_;

        // Hash maps of part typesfirst
        // example of assembly_blue_battery found by 2 logical cameras
        //<assembly_battery_blue, <logical_camera_1, logical_camera_3> >
        std::map<std::string, std::vector<std::string> > logical_camera_map_;
        std::pair<std::string, std::string> logical_camera_1_;
        std::pair<std::string, std::string> logical_camera_3_;
        std::pair<std::string, std::string> logical_camera_2_;
        std::pair<std::string, std::string> logical_camera_4_;

    };
}//namespace

#endif