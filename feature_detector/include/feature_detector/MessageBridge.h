#include "feature_detector/FeaturesFinder.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"

#pragma once

namespace MessageBridge{
  void copyToRosMessage(const FeatureSet& in, mobots_msgs::FeatureSetWithPoseAndID& out);
  void copyToCvStruct(const mobots_msgs::FeatureSetWithPoseAndID& in, FeatureSet& out);
}