#include "feature_detector/FeaturesFinder.h"
#include "mobots_msgs/FeatureSet.h"

#pragma once

namespace MessageBridge{
  void copyToRosMessage(const FeatureSet& in, mobots_msgs::FeatureSet& out);
  void copyToCvStruct(const mobots_msgs::FeatureSet& in, FeatureSet& out);
}