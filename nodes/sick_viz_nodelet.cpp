/***************************************************************************
 * Copyright (C) 2024, Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/

#include "sick_viz/SickViz.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <memory>


namespace sick_safetyscanners
{

class SickVizNodelet : public nodelet::Nodelet
{
public:
    SickVizNodelet() = default;

    ~SickVizNodelet() = default;

    void onInit() override
    {
        ros::NodeHandle nh = getPrivateNodeHandle();

        std::string robot_name;
        std::string laser_name;
        nh.getParam("robot_name", robot_name);
        nh.getParam("laser_name", laser_name);

        sick_viz_protective_ = std::make_unique<sick::SafetyFieldVisualizer>(robot_name, laser_name, false);
        sick_viz_dtz_ = std::make_unique<sick::SafetyFieldVisualizer>(robot_name, laser_name, true);

    }

private:
    std::unique_ptr<sick::SafetyFieldVisualizer> sick_viz_protective_;
    std::unique_ptr<sick::SafetyFieldVisualizer> sick_viz_dtz_;
};

}  // namespace sick_safetyscanners

PLUGINLIB_EXPORT_CLASS(sick_safetyscanners::SickVizNodelet, nodelet::Nodelet)
