#include "competition.h"
#include "utils.h"
#include <std_srvs/Trigger.h>


namespace motioncontrol {
    ////////////////////////
    Competition::Competition(ros::NodeHandle& node) : current_score_(0), logical_camera_map_{}{
        node_ = node;
    }

    ////////////////////////
    void Competition::init() {
        double time_called = ros::Time::now().toSec();
        competition_start_time_ = ros::Time::now().toSec();

        // subscribe to the '/ariac/competition_state' topic.
        competition_state_subscriber_ = node_.subscribe(
            "/ariac/competition_state", 10, &Competition::competitionStateCallback, this);

        // subscribe to the '/clock' topic.
        competition_clock_subscriber_ = node_.subscribe(
            "/clock", 10, &Competition::competitionClockCallback, this);

        // subscribe to the '/ariac/orders' topic.
        competition_order_subscriber_ = node_.subscribe(
            "/ariac/orders", 10, &Competition::competitionOrderCallback, this);
        
        // subscribe to the '/ariac/logical_camera_1' topic.
        competition_logical_camera_1_subscriber_ = node_.subscribe(
            "/ariac/logical_camera_1", 10, &Competition::logicalCamera1Callback, this);

        // subscribe to the '/ariac/logical_camera_2' topic.
        competition_logical_camera_2_subscriber_ = node_.subscribe(
            "/ariac/logical_camera_2", 10, &Competition::logicalCamera2Callback, this);
        
        competition_logical_camera_3_subscriber_ = node_.subscribe(
            "/ariac/logical_camera_3", 10, &Competition::logicalCamera3Callback, this);

        competition_logical_camera_4_subscriber_ = node_.subscribe(
            "/ariac/logical_camera_4", 10, &Competition::logicalCamera4Callback, this);

        // start the competition
        startCompetition();
    }

    // This code will not work if the logical camera detects different parts in different bins
    void Competition::logicalCamera1Callback(const nist_gear::LogicalCameraImage::ConstPtr& msg) {
        // ROS_INFO_STREAM("MAP size: " << logical_camera_map_.size());
        if (msg->models.size() > 0) {
            if (logical_camera_1_.first.compare("") == 0) {
                for (auto const& model : msg->models) {
                    geometry_msgs::Pose pos{};
                    pos = model.pose;
                    
                    if ((pos.position.y > 0))
                    {
                        // ROS_FATAL_STREAM("DETECT");
                        // ROS_FATAL_STREAM(pos.position.y);
                        logical_camera_1_.first = model.type;
                        logical_camera_1_.second = "logical_camera_1";
                    }
                    
                }
            } 
        }
        else {
            logical_camera_1_.first = "";
            logical_camera_1_.second = "";
        }
    }


    void Competition::logicalCamera2Callback(const nist_gear::LogicalCameraImage::ConstPtr& msg) {
        // ROS_INFO_STREAM("MAP size: " << logical_camera_map_.size());
        if (msg->models.size() > 0) {
            if (logical_camera_2_.first.compare("") == 0) {
                for (auto const& model : msg->models) {
                    geometry_msgs::Pose pos{};
                    pos = model.pose;
                    if ((pos.position.y < 0))
                    {
                        logical_camera_2_.first = model.type;
                        logical_camera_2_.second = "logical_camera_2";
                    }
                }
            }
        }
        else {
            logical_camera_2_.first = "";
            logical_camera_2_.second = "";
        }
    }

    void Competition::logicalCamera3Callback(const nist_gear::LogicalCameraImage::ConstPtr& msg) {
        // ROS_INFO_STREAM("MAP size: " << logical_camera_map_.size());
        if (msg->models.size() > 0) {
            if (logical_camera_3_.first.compare("") == 0) {
                for (auto const& model : msg->models) {
                    geometry_msgs::Pose pos{};
                    pos = model.pose;
                    if ((pos.position.y > 0))
                    {
                        logical_camera_3_.first = model.type;
                        logical_camera_3_.second = "logical_camera_3";
                    }
                }
            }
        }
        else {
            logical_camera_3_.first = "";
            logical_camera_3_.second = "";
        }
    }

    void Competition::logicalCamera4Callback(const nist_gear::LogicalCameraImage::ConstPtr& msg) {
        // ROS_INFO_STREAM("MAP size: " << logical_camera_map_.size());
        if (msg->models.size() > 0) {
            if (logical_camera_4_.first.compare("") == 0) {
                for (auto const& model : msg->models) {
                geometry_msgs::Pose pos{};
                    pos = model.pose;
                    if ((pos.position.y < 0))
                    {
                        logical_camera_4_.first = model.type;
                        logical_camera_4_.second = "logical_camera_4";
                    }
                }
            }
        }
        else {
            logical_camera_4_.first = "";
            logical_camera_4_.second = "";
        }
    }
    
    
    ////////////////////////
    void Competition::competitionOrderCallback(const nist_gear::Order::ConstPtr& msg) {
        competition_kitting_shipments_ = msg->kitting_shipments;
        order_id = msg->order_id;
    }

    ////////////////////////
    void Competition::competitionStateCallback(const std_msgs::String::ConstPtr& msg) {
        if (msg->data == "done" && competition_state_ != "done") {
            ROS_INFO("Competition ended.");
        }
        competition_state_ = msg->data;
    }


    ////////////////////////
    void Competition::competitionClockCallback(const rosgraph_msgs::Clock::ConstPtr& msg) {
        competition_clock_ = msg->clock;
    }

    ////////////////////////
    void Competition::startCompetition() {
        // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
        ros::ServiceClient start_client =
            node_.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
        // If it's not already ready, wait for it to be ready.
        // Calling the Service using the client before the server is ready would fail.
        if (!start_client.exists()) {
            start_client.waitForExistence();
        }
        std_srvs::Trigger srv;
        start_client.call(srv);
        if (!srv.response.success) {  // If not successful, print out why.
            ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
        }
        else {
            ROS_INFO("Competition started!");
        }
    }

    ////////////////////////
    void Competition::endCompetition() {
        // Create a Service client for the correct service, i.e. '/ariac/end_competition'.
        ros::ServiceClient end_client =
            node_.serviceClient<std_srvs::Trigger>("/ariac/end_competition");
        // If it's not already ready, wait for it to be ready.
        // Calling the Service using the client before the server is ready would fail.
        if (!end_client.exists()) {
            end_client.waitForExistence();
        }
        std_srvs::Trigger srv;
        end_client.call(srv);
        if (!srv.response.success) {  // If not successful, print out why.
            ROS_ERROR_STREAM("Failed to end the competition: " << srv.response.message);
        }
        else {
            ROS_INFO("Competition ended!");
        }
    }

    ////////////////////////
    double Competition::getStartTime() {
        return competition_start_time_;
    }

    ////////////////////////
    double Competition::getClock() {
        double time_spent = competition_clock_.toSec();
        return time_spent;
    }

    ////////////////////////
    std::string Competition::getCompetitionState() {
        return competition_state_;
    }
}
