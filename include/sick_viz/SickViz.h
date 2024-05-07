#ifndef SICK_SAFETYSCANNERS_SICKVIZ_H
#define SICK_SAFETYSCANNERS_SICKVIZ_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sick_safetyscanners/RawMicroScanDataMsg.h"
#include "sick_safetyscanners/FieldData.h"
#include "sick_safetyscanners/OutputPathsMsg.h"

class SafetyFieldVisualizer {
public:
    SafetyFieldVisualizer(const std::string& robot, const std::string& laser, bool dtz = false);

    void microscanCallback(const sick_safetyscanners::OutputPathsMsg::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::ServiceClient field_data_client_;
    ros::Publisher safety_field_pub_;
    ros::Subscriber raw_data_sub_;
    sensor_msgs::LaserScan current_safety_field_;
    std::string robot_;
    std::string laser_;
    bool dtz_;
    std::string zone_type_;
    sick_safetyscanners::FieldData field_data_;
};

#endif  // SICK_SAFETYSCANNERS_SICKVIZ_H
