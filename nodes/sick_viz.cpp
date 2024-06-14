#include "ros/ros.h"
#include "sick_viz/SickViz.h"

namespace sick 
{

SafetyFieldVisualizer::SafetyFieldVisualizer(const std::string& robot, const std::string& laser, bool dtz)
    : dtz_(dtz), robot_(robot), laser_(laser) {
    // Wait for the service to get the field data
    if (!ros::service::waitForService("/" + robot_ + "/" + laser_ + "_nanoscan/field_data")) {
        ROS_ERROR("Service /%s/%s_nanoscan/field_data not available", robot.c_str(), laser.c_str());
        throw std::runtime_error("Nanoscan field_data service not available");
    }

    field_data_client_ = nh_.serviceClient<sick_safetyscanners::FieldData>("/" + robot_ + "/" + laser_ + "_nanoscan/field_data");
    if (!field_data_client_.call(field_data_)) {
        ROS_ERROR("Failed to call service /%s/%s_nanoscan/field_data", robot.c_str(), laser.c_str());
        throw std::runtime_error("Nanoscan field_data service call failed");
    }

    zone_type_ = (dtz) ? "dtz" : "protective";

    ROS_INFO_STREAM("Number of fields: " << field_data_.response.fields.size());
    if (field_data_.response.fields.empty() ) {
        return;
    }

    safety_field_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/" + robot_ + "/" + laser_ + "_nanoscan/safety_field/" + zone_type_, 10);

    preprocessFieldData();

    // Subscribe to the active monitoring case topic
    raw_data_sub_ = nh_.subscribe("/" + robot_ + "/" + laser_ + "_nanoscan/output_paths", 1, &SafetyFieldVisualizer::microscanCallback, this);
}

void SafetyFieldVisualizer::preprocessFieldData() {
    preprocessed_fields_.clear();

    for (const auto& field : field_data_.response.fields) {
        geometry_msgs::PolygonStamped polygon;
        polygon.header.frame_id = robot_ + "/" + laser_ + "_laser_link";

        for (size_t i = 0; i < field.ranges.size(); ++i) {
            geometry_msgs::Point32 point;
            double angle = field.start_angle + i * field.angular_resolution;
            point.x = field.ranges[i] * cos(angle);
            point.y = field.ranges[i] * sin(angle);
            point.z = 0.0;
            polygon.polygon.points.push_back(point);
        }

        preprocessed_fields_.push_back(polygon);
    }
}

void SafetyFieldVisualizer::microscanCallback(const sick_safetyscanners::OutputPathsMsg::ConstPtr& msg) {
    // If the active_case_index is out of bounds (when the lidars fault) don't publishing anything
    int active_case_index = msg->active_monitoring_case - 1;
    if (active_case_index < 0 || active_case_index >= static_cast<int>(field_data_.response.monitoring_cases.size())) {
        // active_case_index == -1 will occur whenever the SSU faults, no need to log
        if (active_case_index != -1){
            ROS_WARN_STREAM("Invalid active monitoring case: " << active_case_index);
        }
        active_case_index = 0;
    }

    // The field_index query will return 0 if there is no defined field for that monitoring case.
    int field_index = field_data_.response.monitoring_cases[active_case_index].fields[dtz_] - 1;
    if (field_data_.response.monitoring_cases[active_case_index].fields[dtz_] == 0){
        return;
    }
    else{
        field_index = field_data_.response.monitoring_cases[active_case_index].fields[dtz_] - 1;
    }
    if (field_index < 0 || field_index >= static_cast<int>(preprocessed_fields_.size())) {
        throw std::out_of_range("Field index is out of bounds");
    }

    geometry_msgs::PolygonStamped current_safety_field = preprocessed_fields_[field_index];
    current_safety_field.header.stamp = ros::Time::now();

    if (safety_field_pub_.getNumSubscribers() > 0) {
        safety_field_pub_.publish(current_safety_field);
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
        ROS_FATAL("Sick viz crashed for laser %s: %s", laser_name.c_str(), e.what());
        return 1;
    }

    ros::spin();

    return 0;
}
