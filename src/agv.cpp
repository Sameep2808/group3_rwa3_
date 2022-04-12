#include "agv.h"

namespace motioncontrol {
    Agv::Agv(ros::NodeHandle& node, std::string agv_name) : agv_name_{agv_name}
    {
        auto service_client_name = "/ariac/"+agv_name+"/submit_shipment";
        agv_client_ =
            node.serviceClient<nist_gear::AGVToAssemblyStation>(service_client_name);
        // If it's not ready, wait for it to be ready.
        // Calling the Service using the client before the server is ready would fail.
        if (!agv_client_.exists()) {
            ROS_INFO_STREAM("Waiting for " << agv_name << " to be ready...");
            agv_client_.waitForExistence();
            ROS_INFO_STREAM(agv_name << " is now ready.");
        }

        auto agv_state_name = "/ariac/" + agv_name + "/state";
        agv_state_subscriber_ = node.subscribe(agv_state_name, 10, &Agv::agv_state_callback, this);
    }


    bool Agv::shipAgv(std::string shipment_type, std::string station) {
        nist_gear::AGVToAssemblyStation msg;
        msg.request.assembly_station_name = station;
        msg.request.shipment_type = shipment_type;
        agv_client_.call(msg);


        if (msg.response.success) {
            ROS_INFO_STREAM("[agv_control][sendAGV] AGV is taking order: " + msg.request.shipment_type);
            return true;
        }
        else {
            ROS_ERROR_STREAM("[agv_control][sendAGV] Failed to call AGV!");
            return false;
        }
    }

    bool Agv::getAGVStatus()
    {
        return agv_ready_;
    }

    void Agv::agv_state_callback(const std_msgs::String& msg)
    {
        if (!((msg.data).compare("ready_to_deliver")))
            agv_ready_ = true;
        else
            agv_ready_ = false;
    }
}//namespace
