#include <fstream>
#include <sys/stat.h>
#include "mobots_common/utils.h"
#include "feature.h"
#include "image_pose_data_types.h"

namespace ser = ros::serialization;

bool FeatureStore::loadFeatureSet(const IDT& idt, mobots_msgs::FeatureSetWithPoseAndID& featureSet){
  std::string path = mobots_common::store::getPathForID(idt.sessionID, idt.mobotID, idt.imageID, ".feature");
  struct stat filestatus;
  stat(path.c_str(), &filestatus);
  long size = filestatus.st_size;
  std::ifstream in;
  in.open(path.c_str(), std::ios::binary);
  if(!in)
	 return false;
  unsigned char* data = new unsigned char[size];
  in.read((char*) data, size);
  ser::IStream stream(data, size);
  ser::deserialize(stream, featureSet);
  in.close();
  delete[] data;
  return true;
}

bool FeatureStore::saveFeatureSet(const mobots_msgs::FeatureSetWithPoseAndID& featureSet){
  uint32_t serial_size = ser::serializationLength(featureSet);
  unsigned char* data = new unsigned char[serial_size];
  ser::OStream stream(data, serial_size);
  ser::serialize(stream, featureSet);
  IDT idt;
  idt.mobotID = featureSet.id.mobot_id;
  idt.sessionID = featureSet.id.session_id;
  idt.imageID = featureSet.id.image_id;
  std::ofstream out;
  out.open(mobots_common::store::getPathForID(idt.sessionID, idt.mobotID, idt.imageID, ".feature").c_str(), std::ios::binary);
  if(!out)
	 return false;
  out.write((const char*) data, serial_size);
  out.flush();
  out.close();
  delete[] data;
  return true;
}
