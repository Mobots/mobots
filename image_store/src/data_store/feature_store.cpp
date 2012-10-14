#include <fstream>

#include "mobots_msgs/FeatureSetWithPoseAndID.h"
#include "image_info_data_types.h"

namespace ser = ros::serialization;

std::string getPathForIDT(const IDT id, const std::string& fileEnding){
  std::stringstream ss;
  ss << "~/session-" << id.sessionID << "/mobot-" << id.mobotID << "-" << id.imageID << fileEnding;
  return ss.str();
}

bool loadFeatureSet(const IDT mobotID, mobots_msgs::FeatureSetWithPoseAndID& featureSet){
  uint32_t serial_size = ros::serialization::serializationLength(featureSet);
  std::ifstream in(getPathForIDT(mobotID, "").c_str(), std::ios::binary);
  unsigned char* data = new unsigned char[serial_size];
  in.read((char*) data, serial_size);
  ser::IStream stream(data, serial_size);
  ser::deserialize(stream, featureSet);
  in.close();
  delete[] data;
  return true;
}

bool saveFeatureSet(const mobots_msgs::FeatureSetWithPoseAndID& featureSet){
  uint32_t serial_size = ros::serialization::serializationLength(featureSet);
  unsigned char* data = new unsigned char[serial_size];
  ser::OStream stream(data, serial_size);
  ser::serialize(stream, featureSet);
  IDT idt;
  idt.mobotID = featureSet.id.mobot_id;
  idt.sessionID = featureSet.id.session_id;
  idt.imageID = featureSet.id.image_id;
  std::ofstream out(getPathForIDT(idt, "").c_str(), std::ios::binary);
  out.write((const char*) data, serial_size);
  out.close();
  delete[] data;
  return true;
}