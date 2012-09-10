#include "feature_detector/FeaturesFinder.h"
#include "mobots_msgs/FeatureSetWithDeltaPoseAndID.h"

#pragma once

namespace MessageBridge{
  void copyToRosMessage(const FeatureSet& in, mobots_msgs::FeatureSetWithDeltaPoseAndID& out);
  void copyToCvStruct(const mobots_msgs::FeatureSetWithDeltaPoseAndID& in, FeatureSet& out);
}