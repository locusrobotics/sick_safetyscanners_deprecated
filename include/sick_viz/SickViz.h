#ifndef SICK_SAFETYSCANNERS_SICKVIZ_H
#define SICK_SAFETYSCANNERS_SICKVIZ_H

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "sick_safetyscanners/RawMicroScanDataMsg.h"
#include "sick_safetyscanners/FieldData.h"
#include "sick_safetyscanners/OutputPathsMsg.h"
#include <dynamic_reconfigure/server.h>
#include <sick_safetyscanners/SickSafetyscannersConfigurationConfig.h>
#include <vector>

namespace sick {
/**
* @brief SafetyFieldVisualizer object that takes in a lidar name and publishes safety fields
*/
class SafetyFieldVisualizer {
public:
    SafetyFieldVisualizer(const std::string& robot, const std::string& laser, bool dtz = false);

    void microscanCallback(const sick_safetyscanners::OutputPathsMsg::ConstPtr& msg);
    void dynamicReconfigCallback(const dynamic_reconfigure::Config::ConstPtr& msg);

private:
    void preprocessFieldData();
    void downsampleMarkerPoints(visualization_msgs::Marker& marker, float epsilon);

    ros::NodeHandle nh_;
    ros::ServiceClient field_data_client_;
    ros::Publisher safety_field_pub_;
    ros::Publisher monitoring_case_pub_;
    ros::Subscriber raw_data_sub_;
    ros::Subscriber dyn_reconf_sub_;
    sick_safetyscanners::FieldData field_data_;
    std::vector<visualization_msgs::Marker> preprocessed_markers_;
    visualization_msgs::Marker monitoring_case_marker_;
    std::string robot_;
    std::string laser_;
    std::string zone_type_;
    bool dtz_;
    int polygon_size_;
    
};

}  // namespace sick

#endif  // SICK_SAFETYSCANNERS_SICKVIZ_H
