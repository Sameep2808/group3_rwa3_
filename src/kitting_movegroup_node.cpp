//ros
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
//moveit
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
//standard library
#include <math.h>
#include <vector>
//custom
#include "competition.h"
#include "arm.h"
#include "agv.h"
#include "utils.h"
#include "nist_gear/Product.h"

void ship_agv(ros::NodeHandle& nh, const nist_gear::KittingShipment& kitting_shipment)
{
    ROS_INFO_STREAM("Shipping AGV " << kitting_shipment.agv_id);
    ros::Duration(sleep(1.0));
    motioncontrol::Agv agv{ nh, kitting_shipment.agv_id };
    if (agv.getAGVStatus()) {
        agv.shipAgv(kitting_shipment.shipment_type, kitting_shipment.station_id);
        ros::Duration(2.0).sleep();
    }
    ros::Duration(2.0).sleep();
}

int kit(ros::NodeHandle& node_handle, std::vector<std::pair<std::string, int>> &product_list, bool* blackout_active, int P){

    // start the competition
    motioncontrol::Competition competition(node_handle);
    competition.init();


    // create an instance of the kitting arm
    motioncontrol::Arm arm(node_handle);
    arm.init();

    // Wait for order to be recieved
    ros::Rate rate = 2;
    while (competition.getKittingShipments().size() == 0) {
        ROS_INFO_STREAM("Waiting for order");
        rate.sleep();
    }

    // if we get to this point it means we started receiving orders
    auto kitting_shipments = competition.getKittingShipments();
    auto orderid = competition.get_order_id();
    arm.goToPresetLocation("home1");
    arm.goToPresetLocation("home2");

    // this variable will be used to build frame names (see line 142)
    int counter{};
    // parse each kitting shipment

    // std::vector<std::pair<std::string, int>> product_list{};

    std::vector<nist_gear::KittingShipment> queued_shipments;

    while(1){
    
    for (auto& kitting_shipment : kitting_shipments) {
        // counter++;
        // list of different product types
        // if the shipment has 2 blue batteries then only 1 blue
        // battery is placed in this list.
        // the list will look like this:
        // [('assembly_pump_red', nist_gear::Product), ('assembly_battery_blue', nist_gear::Product)]
        // std::vector<std::pair<std::string, nist_gear::Product>> product_list{};
        
        for (const auto& product : kitting_shipment.products) {
            bool flag = true;
            if (product_list.empty()){
                std::pair< std::string, int> entry;
                entry.first = product.type;
                entry.second = 0;
                product_list.emplace_back(entry);
                flag = false;
            }
            else {
                for (const auto& le : product_list ){
                    if (le.first.compare(product.type)==0)
                    {
                        flag = false;
                        break;
                    }
                }
            }
            if (flag){
                std::pair< std::string, int> entry;
                entry.first = product.type;
                entry.second = 0;
                product_list.emplace_back(entry);
            }   
        }

        // for(auto p : product_list){
        //     ROS_FATAL_STREAM(p.first);
        // }

        // Let's wait 10 s
        // data should be publishing after 5 seconds
        double outside_time = ros::Time::now().toSec();
        double inside_time = ros::Time::now().toSec();
        while (inside_time - outside_time < 5.0) {
            inside_time = ros::Time::now().toSec();
            // ROS_INFO_STREAM("Waiting for data");
        }
        
        auto camera1_data = competition.get_logical_camera1_data();
        auto camera2_data = competition.get_logical_camera2_data();
        auto camera3_data = competition.get_logical_camera3_data();
        auto camera4_data = competition.get_logical_camera4_data();
        // ROS_FATAL_STREAM(camera1_data.first);
        // ROS_FATAL_STREAM(camera1_data.second);
        // ROS_FATAL_STREAM(camera2_data.first);
        // ROS_FATAL_STREAM(camera2_data.second);
        // ROS_FATAL_STREAM(camera3_data.first);
        // ROS_FATAL_STREAM(camera3_data.second);
        // ROS_FATAL_STREAM(camera4_data.first);
        // ROS_FATAL_STREAM(camera4_data.second);
        if ((camera1_data.first.compare("") == 0) && (camera2_data.first.compare("") == 0)  && (camera3_data.first.compare("") == 0)  && (camera4_data.first.compare("") == 0)) {
            ROS_FATAL_STREAM("No data reported by any logical camera");
            ros::shutdown();
            return 1;
        }

        ROS_INFO_STREAM("Received data from camera");
        
        std::vector<std::pair<nist_gear::Product, std::string> > camera_for_product{};
        // If we got here it means cameras got some data
        // check which camera found products needed in this shipment
        if(P == 0){
        for (const auto& product : kitting_shipment.products) {
            if (camera2_data.first.compare(product.type) == 0) {
                std::pair<nist_gear::Product, std::string> entry;
                entry.first = product;
                entry.second = camera2_data.second;
                camera_for_product.emplace_back(entry);
            }
            else if (camera3_data.first.compare(product.type) == 0) {
                std::pair<nist_gear::Product, std::string> entry;
                entry.first = product;
                entry.second = camera3_data.second;
                camera_for_product.emplace_back(entry);
            }
            else if (camera1_data.first.compare(product.type) == 0) {
                std::pair<nist_gear::Product, std::string> entry;
                entry.first = product;
                entry.second = camera1_data.second;
                camera_for_product.emplace_back(entry);
            }
            else if (camera4_data.first.compare(product.type) == 0) {
                std::pair<nist_gear::Product, std::string> entry;
                entry.first = product;
                entry.second = camera4_data.second;
                camera_for_product.emplace_back(entry);
            }
            else {
                ROS_FATAL_STREAM("No matching part found by any logical camera");
                ros::shutdown();
                return 1;
            }
        }}
        else{
        for (const auto& product : kitting_shipment.products) {
            if (camera4_data.first.compare(product.type) == 0) {
                std::pair<nist_gear::Product, std::string> entry;
                entry.first = product;
                entry.second = camera4_data.second;
                camera_for_product.emplace_back(entry);
            }
            else if (camera1_data.first.compare(product.type) == 0) {
                std::pair<nist_gear::Product, std::string> entry;
                entry.first = product;
                entry.second = camera1_data.second;
                camera_for_product.emplace_back(entry);
            }
            else if (camera3_data.first.compare(product.type) == 0) {
                std::pair<nist_gear::Product, std::string> entry;
                entry.first = product;
                entry.second = camera3_data.second;
                camera_for_product.emplace_back(entry);
            }
            else if (camera2_data.first.compare(product.type) == 0) {
                std::pair<nist_gear::Product, std::string> entry;
                entry.first = product;
                entry.second = camera2_data.second;
                camera_for_product.emplace_back(entry);
            }
            else {
                ROS_FATAL_STREAM("No matching part found by any logical camera");
                ros::shutdown();
                return 1;
            }
        }}

        if (camera_for_product.empty()) {
            ROS_FATAL_STREAM("Product not found by camera");
            ros::shutdown();
            return 1;
        }

        // keep track of how many products have been placed in this shipment
        int product_placed_in_shipment{};
        for (auto iter : camera_for_product) {
            
            int* co;
            for (auto& p : product_list){
                if(p.first.compare(iter.first.type) == 0)
                {
                    co = &p.second;
                    p.second = p.second + 1; 
                    // ROS_FATAL_STREAM(p.second);
                }
            }
            arm.counter = co;
            std::string part_frame = iter.second + "_" + iter.first.type + "_" + std::to_string(*co)+ "_frame";
            
            ROS_INFO_STREAM("Kitting node frame: " << part_frame);
            arm.movePart(iter.first.type, part_frame, iter.first.pose, kitting_shipment.agv_id, blackout_active);
            product_placed_in_shipment++;
            // if we have placed all products in this shipment then ship the AGV
            if (product_placed_in_shipment == kitting_shipment.products.size()) {
                if (*blackout_active) {
                    queued_shipments.push_back(kitting_shipment);
                    ROS_INFO_STREAM("Queued kitting shipment for AGV " << kitting_shipment.agv_id);
                } else {
                    ship_agv(node_handle, kitting_shipment);
                }
            }
        }

        auto on = competition.get_order_id();
        if(orderid.compare(on) != 0){
                ROS_FATAL_STREAM("HIGH PRIORITY ORDER");
                ROS_FATAL_STREAM(on);
                orderid = on;   
                ros::NodeHandle nh;
                int fr = kit(nh, product_list, blackout_active, 0); 
        }
       
        
    }

    if (arm.hasQueuedParts())
    {
        // TODO: blatant race condition...
        while (*blackout_active)
        {
            ros::Duration(0.5).sleep();
        }
        arm.finalizeQueuedParts();
        while (!queued_shipments.empty())
        {
            ship_agv(node_handle, *(queued_shipments.end()));
            queued_shipments.pop_back();
        }
    }

    auto on = competition.get_order_id();
    if(orderid.compare(on) != 0){
            ROS_FATAL_STREAM(on);
            kitting_shipments.clear();
            kitting_shipments = competition.getKittingShipments();
            orderid = on;
            for (auto ks : kitting_shipments){
                ROS_FATAL_STREAM(ks.agv_id);
            }  
        }
        else{
            break;
        }
    }

    return 0;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "rwa3_node_cpp");
    ros::NodeHandle node_handle;

    // Utilities to monitor for a sensor blackout and react to it
    bool blackout_active = false;
    bool sensors_started = false;
    ros::Time last_sensor_update;
    ros::Timer watch_for_blackouts_tmr = node_handle.createTimer(
        ros::Duration(1.0),
        [&](const ros::TimerEvent& evt) {
            // Assume this isn't a blackout if we haven't heard from any sensor
            // ever and that the system is still starting up
            if (sensors_started && !blackout_active)
            {
                // If we haven't heard from a sensor in over a second, assume
                // we're in a blackout
                blackout_active = (ros::Time::now()-last_sensor_update).toSec() > 1.0;
                if (blackout_active)
                {
                    ROS_ERROR_STREAM("Sensor blackout detected!");
                }
            }
        }
    );
    const auto logical_camera_image_callback = [&](const nist_gear::LogicalCameraImage::ConstPtr& msg) {
        // This just says we have heard from any sensor throughout the lifetime
        // of the application
        sensors_started = true;
        // Right now is the most recent update we've heard from a sensor
        last_sensor_update = ros::Time::now();
        // If we're in a blackout, we're exiting it
        if (blackout_active)
        {
            ROS_INFO_STREAM("Sensor blackout ended.");
            blackout_active = false;
        }
    };
    const std::vector<std::string> logical_camera_image_topics = {
        "/ariac/logical_camera_bins0",
        "/ariac/logical_camera_bins1",
        "/ariac/logical_camera_bins2",
        "/ariac/logical_camera_bins3",
        "/ariac/quality_control_sensor_0",
        "/ariac/quality_control_sensor_1",
        "/ariac/quality_control_sensor_2",
        "/ariac/quality_control_sensor_3"
    };
    std::vector<ros::Subscriber> logical_camera_image_subs;
    for (auto topic_iter = logical_camera_image_topics.cbegin();
         topic_iter != logical_camera_image_topics.cend();
         ++topic_iter)
    {
        logical_camera_image_subs.push_back(
            node_handle.subscribe<nist_gear::LogicalCameraImage>(
                *topic_iter,
                1,
                logical_camera_image_callback
            )
        );
    }

    // 0 means that the spinner will use as many threads as there are processors on your machine.
    //If you use 3 for example, only 3 threads will be used.
    ros::AsyncSpinner spinner(0);
    spinner.start();
    std::vector<std::pair<std::string, int>> product_list{};
    int r = kit(node_handle, product_list, &blackout_active, 0);
    
    // competition.endCompetition();
    ros::waitForShutdown();

    return r;
}
