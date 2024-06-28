#include <geometry_msgs/Point.h>
#include "ros/ros.h"
#include "sick_viz/SickViz.h"
#include "sick_viz/DouglasPeucker.h"

namespace sick 
{

SafetyFieldVisualizer::SafetyFieldVisualizer(const std::string& robot, const std::string& laser, bool dtz)
    : robot_(robot), laser_(laser), dtz_(dtz), epsilon_(0.8) {
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

    safety_field_pub_ = nh_.advertise<visualization_msgs::Marker>("/" + robot_ + "/" + laser_ + "_scan/safety_field/" + zone_type_, 10);
    monitoring_case_pub_ = nh_.advertise<visualization_msgs::Marker>("/" + robot_ + "/" + laser_ + "_scan/monitoring_case_marker", 10);

    preprocessFieldData();

    // Subscribe to the active monitoring case topic
    raw_data_sub_ = nh_.subscribe("/" + robot_ + "/" + laser_ + "_nanoscan/output_paths", 1, &SafetyFieldVisualizer::microscanCallback, this);

    // Dynamic reconfigure server setup
    dynamic_reconfigure::Server<sick_safetyscanners::SickVizConfig>::CallbackType cb;
    cb = boost::bind(&SafetyFieldVisualizer::dynamicReconfigCallback, this, _1, _2);
    dr_srv_.setCallback(cb);
}

void simplifyMarkerPoints(visualization_msgs::Marker& marker, float epsilon) {
    std::vector<dougpeuck::Point> inputPoints, simplifiedPoints;

    for (const auto& pt : marker.points) {
        inputPoints.emplace_back(pt.x, pt.y);
    }

    simplifiedPoints = dougpeuck::simplifyPolyline(inputPoints, epsilon);

    marker.points.clear();
    for (const auto& pt : simplifiedPoints) {
        geometry_msgs::Point rosPoint;
        rosPoint.x = pt.x;
        rosPoint.y = pt.y;
        rosPoint.z = 0.0;
        marker.points.push_back(rosPoint);
    }
}

void SafetyFieldVisualizer::preprocessFieldData() {
    preprocessed_markers_.clear();
    int marker_count = 0;

    for (const auto& field : field_data_.response.fields) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = robot_ + "/" + laser_ + "_laser_link";
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.a = 1.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        if (dtz_){
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else{
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }

        for (size_t i = 0; i < field.ranges.size(); ++i) {
            geometry_msgs::Point point;
            double angle = field.start_angle + i * field.angular_resolution;
            point.x = field.ranges[i] * cos(angle);
            point.y = field.ranges[i] * sin(angle);
            point.z = 0.0;
            marker.points.push_back(point);
        }

        simplifyMarkerPoints(marker, epsilon_);
        preprocessed_markers_.push_back(marker);
        marker_count += marker.points.size();
    }
    float average_size = marker_count / field_data_.response.fields.size();
    ROS_INFO("Douglas Peucker Downsampling - %s %s average field marker count: %f epsilon: %f",
              laser_.c_str(), zone_type_.c_str(), average_size, epsilon_);

    // Initialize monitoring_case_marker_
    monitoring_case_marker_.header.frame_id = robot_ + "/" + laser_ + "_laser_link";
    monitoring_case_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    monitoring_case_marker_.action = visualization_msgs::Marker::ADD;
    monitoring_case_marker_.color.a = 1.0;
    monitoring_case_marker_.pose.orientation.w = 1.0;
    monitoring_case_marker_.scale.x = 0.25;
    monitoring_case_marker_.scale.y = 0.25;
    monitoring_case_marker_.scale.z = 0.25;
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
    if (field_index < 0 || field_index >= static_cast<int>(preprocessed_markers_.size())) {
        throw std::out_of_range("Field index is out of bounds");
    }

    visualization_msgs::Marker current_safety_field = preprocessed_markers_[field_index];

    if (!msg->status[dtz_]) {
        current_safety_field.color.r = 1.0;
        current_safety_field.color.g = 0.0;
        current_safety_field.color.b = 0.0;
        monitoring_case_marker_.color.r = 1.0;
        monitoring_case_marker_.color.g = 0.0;
        monitoring_case_marker_.color.b = 0.0;
    }
    else{
        monitoring_case_marker_.color.r = 0.0;
        monitoring_case_marker_.color.g = 1.0;
        monitoring_case_marker_.color.b = 0.0;
    }
    monitoring_case_marker_.text = std::to_string(msg->active_monitoring_case);

    current_safety_field.header.stamp = ros::Time::now();
    monitoring_case_marker_.header.stamp = ros::Time::now();

    if (safety_field_pub_.getNumSubscribers() > 0) {
        safety_field_pub_.publish(current_safety_field);
    }

    if (monitoring_case_pub_.getNumSubscribers() > 0) {
        monitoring_case_pub_.publish(monitoring_case_marker_);
    }
}

void SafetyFieldVisualizer::dynamicReconfigCallback(sick_safetyscanners::SickVizConfig &config, uint32_t level) {
    epsilon_ = config.epsilon;
    preprocessFieldData();  // Re-process the field data with the new epsilon value
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
