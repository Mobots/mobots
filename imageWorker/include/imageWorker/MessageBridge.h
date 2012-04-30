#pragma once

namespace MessageBridge{
  void copyToRosMessage(const ImageFeatures& in, mobots_msgs::FeatureSetWithDeltaPose& out);
  void copyToCvStruct(const mobots_msgs::FeatureSetWithDeltaPose& in, ImageFeatures& out);
}