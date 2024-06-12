#include "ros/ros.h"
#include "sick_viz/SickViz.h"

namespace sick 
{

SafetyFieldVisualizer::SafetyFieldVisualizer(const std::string& robot, const std::string& laser, bool dtz)
    : dtz_(dtz) {
    // Wait for the service to get the field data
    if (!ros::service::waitForService("/" + robot + "/" + laser + "_nanoscan/field_data", ros::Duration(5))) {
        ROS_ERROR("Service /%s/%s_nanoscan/field_data not available", robot.c_str(), laser.c_str());
        throw std::runtime_error("Nanoscan field_data service not available");
    }

    field_data_client_ = nh_.serviceClient<sick_safetyscanners::FieldData>("/" + robot + "/" + laser + "_nanoscan/field_data");
    if (!field_data_client_.call(field_data_)) {
        ROS_ERROR("Failed to call service /%s/%s_nanoscan/field_data", robot.c_str(), laser.c_str());
        throw std::runtime_error("Nanoscan field_data service call failed");
    }

    zone_type_ = (dtz) ? "dtz" : "protective";

    ROS_INFO_STREAM("Number of fields: " << field_data_.response.fields.size());
    if (field_data_.response.fields.empty() ) {
        return;
    }

    safety_field_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/" + robot + "/" + laser + "_nanoscan/safety_field/" + zone_type_, 10);
    current_safety_field_.header.frame_id = robot + "/" + laser + "_laser_link";
    current_safety_field_.angle_min = field_data_.response.fields[dtz].start_angle;
    current_safety_field_.angle_max = field_data_.response.fields[dtz].start_angle + field_data_.response.fields[dtz].angular_resolution * field_data_.response.fields[dtz].ranges.size();
    current_safety_field_.angle_increment = field_data_.response.fields[dtz].angular_resolution;
    current_safety_field_.range_min = 0.01;
    current_safety_field_.range_max = 8.0;

    // Subscribe to the active monitoring case topic
    raw_data_sub_ = nh_.subscribe("/" + robot + "/" + laser + "_nanoscan/output_paths", 1, &SafetyFieldVisualizer::microscanCallback, this);
}

void SafetyFieldVisualizer::microscanCallback(const sick_safetyscanners::OutputPathsMsg::ConstPtr& msg) {
    current_safety_field_.header.stamp = ros::Time::now();

    if (msg->active_monitoring_case == 0)
    {
        return;
    }

    current_safety_field_.ranges = field_data_.response.fields[field_data_.response.monitoring_cases[msg->active_monitoring_case - 1].fields[dtz_]-1].ranges;

    if (safety_field_pub_.getNumSubscribers() > 0) {
        safety_field_pub_.publish(current_safety_field_);
    }
}

}  // namespace sick

int main(int argc, char** argv) {
    ros::init(argc, argv, "sick_viz");
    ros::NodeHandle nh("~");

    std::string robot_name;
    std::string laser_name;

    nh.getParam("robot_name", robot_name);
    nh.getParam("laser_name", laser_name);

    try {
        sick::SafetyFieldVisualizer protective(robot_name, laser_name, false);
        sick::SafetyFieldVisualizer DTZ(robot_name, laser_name, true);
    } catch (const std::runtime_error& e) {
        ROS_ERROR("Sick viz crashed for laser %s: %s", laser_name.c_str(), e.what());
        return 1;
    }

    ros::spin();

    return 0;
}
