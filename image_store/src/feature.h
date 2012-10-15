#include "mobots_msgs/FeatureSetWithPoseAndID.h"
#include "image_pose_data_types.h"

#pragma once

/**
 * Provides functions to save and load feature sets to/from disk
 */
namespace FeatureStore{

/**
  * Attempts to load a feature set for the given IDT struct.
  * The result is stored in the featureSet parameter.
  * Returns false if no featureSet could be loaded for the given IDT.
  */
bool loadFeatureSet(const IDT& idt, mobots_msgs::FeatureSetWithPoseAndID& featureSet);

/**
 * Attempts to write a feature set to disk.
 * Returns false on failure to do so.
 */
bool saveFeatureSet(const mobots_msgs::FeatureSetWithPoseAndID& featureSet);

};