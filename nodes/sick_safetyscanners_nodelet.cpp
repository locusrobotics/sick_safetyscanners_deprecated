/***************************************************************************
 * Copyright (C) 2024, Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/

#include "sick_safetyscanners/SickSafetyscannersRos.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <memory>


namespace sick_safetyscanners
{

class SickSafetyscannersNodelet : public nodelet::Nodelet
{
public:
    SickSafetyscannersNodelet() = default;

    ~SickSafetyscannersNodelet() = default;

    void onInit() override
    {
        sick_safetyscanners_ = std::make_unique<sick::SickSafetyscannersRos>(getPrivateNodeHandle());
    }

private:
    std::unique_ptr<sick::SickSafetyscannersRos> sick_safetyscanners_;
};

}  // namespace sick_safetyscanners

PLUGINLIB_EXPORT_CLASS(sick_safetyscanners::SickSafetyscannersNodelet, nodelet::Nodelet)
