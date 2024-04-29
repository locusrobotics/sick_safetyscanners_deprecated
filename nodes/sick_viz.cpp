#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sick_safetyscanners/RawMicroScanDataMsg.h"  
#include "sick_safetyscanners/FieldData.h"
#include "sick_safetyscanners/OutputPathsMsg.h"

class SafetyFieldVisualizer {
public:
    SafetyFieldVisualizer(const std::string& robot, const std::string& laser, int augment = 0)
        : robot_(robot), laser_(laser), augment_(augment) {
        // Wait for the service to get the field data
        ros::service::waitForService("/" + robot + "/" + laser + "_nanoscan/field_data", ros::Duration(5));
        field_data_client_ = nh_.serviceClient<sick_safetyscanners::FieldData>("/" + robot + "/" + laser + "_nanoscan/field_data");
        field_data_client_.call(field_data_);
        zone_type_ = (augment) ? "protective":"DTZ";

        ROS_INFO_STREAM("Number of fields: " << field_data_.response.fields.size());
        safety_field_pub_ = nh_.advertise
        <sensor_msgs::LaserScan>("/" + robot + "/" + laser + "_nanoscan/safety_field/" + zone_type_, 10);
        current_safety_field_.header.frame_id = robot + "/" + laser + "_laser_link";
        current_safety_field_.angle_min = field_data_.response.fields[augment].start_angle;
        current_safety_field_.angle_max = field_data_.response.fields[augment].start_angle + field_data_.response.fields[augment].angular_resolution * field_data_.response.fields[augment].ranges.size();
        current_safety_field_.angle_increment = field_data_.response.fields[augment].angular_resolution;
        // current_safety_field_.ranges = field_data.response.fields[augment].ranges;
        current_safety_field_.range_min = 0.01;
        current_safety_field_.range_max = 8.0;

        // Subscribe to the raw microscan data
        raw_data_sub_ = nh_.subscribe("/" + robot + "/" + laser + "_nanoscan/output_paths", 1, &SafetyFieldVisualizer::microscanCallback, this);
    }

    void microscanCallback(const sick_safetyscanners::OutputPathsMsg::ConstPtr& msg) {
        current_safety_field_.header.stamp = ros::Time::now();
        current_safety_field_.ranges = field_data_.response.fields[msg->active_monitoring_case + augment_].ranges;
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
    int augment_;
    std::string zone_type_;
    sick_safetyscanners::FieldData field_data_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sick_viz");

    std::string robot_name = "v2_19813";
    SafetyFieldVisualizer left_protective(robot_name, "left");
    SafetyFieldVisualizer right_protective(robot_name, "right");
    SafetyFieldVisualizer rear_protective(robot_name, "rear");
    SafetyFieldVisualizer left_DTZ(robot_name, "left", 1);
    SafetyFieldVisualizer right_DTZ(robot_name, "right", 1);
    SafetyFieldVisualizer rear_DTZ(robot_name, "rear", 1);

    ros::spin();

    return 0;
}