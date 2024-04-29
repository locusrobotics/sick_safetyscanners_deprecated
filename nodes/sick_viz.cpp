#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sick_safetyscanners/RawMicroScanDataMsg.h"  
#include "sick_safetyscanners/FieldData.h"
#include "sick_safetyscanners/OutputPathsMsg.h"

class SafetyFieldVisualizer {
public:
    SafetyFieldVisualizer(const std::string& robot, const std::string& laser, bool dtz = false)
        : robot_(robot), laser_(laser), dtz_(dtz) {
        // Wait for the service to get the field data
        ros::service::waitForService("/" + robot + "/" + laser + "_nanoscan/field_data", ros::Duration(5));
        field_data_client_ = nh_.serviceClient<sick_safetyscanners::FieldData>("/" + robot + "/" + laser + "_nanoscan/field_data");
        field_data_client_.call(field_data_);
        zone_type_ = (dtz) ? "DTZ":"protective";

        ROS_INFO_STREAM("Number of fields: " << field_data_.response.fields.size());
        safety_field_pub_ = nh_.advertise
        <sensor_msgs::LaserScan>("/" + robot + "/" + laser + "_nanoscan/safety_field/" + zone_type_, 10);
        current_safety_field_.header.frame_id = robot + "/" + laser + "_laser_link";
        current_safety_field_.angle_min = field_data_.response.fields[dtz].start_angle;
        current_safety_field_.angle_max = field_data_.response.fields[dtz].start_angle + field_data_.response.fields[dtz].angular_resolution * field_data_.response.fields[dtz].ranges.size();
        current_safety_field_.angle_increment = field_data_.response.fields[dtz].angular_resolution;
        current_safety_field_.range_min = 0.01;
        current_safety_field_.range_max = 8.0;

        // Subscribe to the active monitoring case
        raw_data_sub_ = nh_.subscribe("/" + robot + "/" + laser + "_nanoscan/output_paths", 1, &SafetyFieldVisualizer::microscanCallback, this);
    }

    void microscanCallback(const sick_safetyscanners::OutputPathsMsg::ConstPtr& msg) {
        current_safety_field_.header.stamp = ros::Time::now();
        current_safety_field_.ranges = field_data_.response.fields[field_data_.response.monitoring_cases[msg->active_monitoring_case-1].fields[dtz_]-1].ranges;
      
        safety_field_pub_.publish(current_safety_field_);
    }

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "sick_viz");

    std::string robot_name = "v2_19814";
    SafetyFieldVisualizer left_protective(robot_name, "left", false);
    SafetyFieldVisualizer right_protective(robot_name, "right", false);
    SafetyFieldVisualizer rear_protective(robot_name, "rear", false);
    SafetyFieldVisualizer left_DTZ(robot_name, "left", true);
    SafetyFieldVisualizer right_DTZ(robot_name, "right", true);
    SafetyFieldVisualizer rear_DTZ(robot_name, "rear", true);

    ros::spin();

    return 0;
}