#include "ros/ros.h"
#include "sick_viz/SickViz.h"


SafetyFieldVisualizer::SafetyFieldVisualizer(const std::string& robot, const std::string& laser, bool dtz)
    : robot_(robot), laser_(laser), dtz_(dtz) {
    // Wait for the service to get the field data
    ros::service::waitForService("/" + robot + "/" + laser + "_nanoscan/field_data", ros::Duration(5));
    field_data_client_ = nh_.serviceClient<sick_safetyscanners::FieldData>("/" + robot + "/" + laser + "_nanoscan/field_data");
    field_data_client_.call(field_data_);
    zone_type_ = (dtz) ? "DTZ" : "protective";

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
    current_safety_field_.ranges = field_data_.response.fields[field_data_.response.monitoring_cases[msg->active_monitoring_case - 1].fields[dtz_]-1].ranges;

    if (safety_field_pub_.getNumSubscribers() > 0) {
        safety_field_pub_.publish(current_safety_field_);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sick_viz");
    ros::NodeHandle nh("~");

    std::string robot_name;
    std::string laser_name;

    nh.getParam("robot_name", robot_name);
    nh.getParam("laser_name", laser_name);

    SafetyFieldVisualizer protective(robot_name, laser_name, false);
    SafetyFieldVisualizer DTZ(robot_name, laser_name, true);

    ros::spin();

    return 0;
}
