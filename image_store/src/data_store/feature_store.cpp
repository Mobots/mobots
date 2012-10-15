#include <fstream>

#include "mobots_msgs/FeatureSetWithPoseAndID.h"
#include "image_info_data_types.h"

namespace ser = ros::serialization;

const char* HOME = getenv("HOME");

std::string getPathForIDT(const IDT id, const std::string& fileEnding){
  std::stringstream ss;
  ss << HOME << "/session-" << id.sessionID << "/mobot-" << id.mobotID << "-" << id.imageID << fileEnding;
  return ss.str();
}

namespace FeatureStore{
	int size;

bool loadFeatureSet(const IDT idt, mobots_msgs::FeatureSetWithPoseAndID& featureSet){
  uint32_t serial_size = ros::serialization::serializationLength(featureSet);
  std::cout << "load  path " << getPathForIDT(idt, "") << std::endl;
  std::ifstream in(getPathForIDT(idt, "").c_str(), std::ios::binary);
  unsigned char* data = new unsigned char[size];
  in.read((char*) data, size);
  ser::IStream stream(data, size);
  ser::deserialize(stream, featureSet);
  in.close();
  delete[] data;
  return true;
}

bool saveFeatureSet(const mobots_msgs::FeatureSetWithPoseAndID& featureSet){
  uint32_t serial_size = ros::serialization::serializationLength(featureSet);
  size = serial_size;
  unsigned char* data = new unsigned char[serial_size];
  ser::OStream stream(data, serial_size);
  ser::serialize(stream, featureSet);
  IDT idt;
  idt.mobotID = featureSet.id.mobot_id;
  idt.sessionID = featureSet.id.session_id;
  idt.imageID = featureSet.id.image_id;
  std::ofstream out;
  out.open(getPathForIDT(idt, "").c_str(), std::ios::binary);
  std::cout << "save path " << getPathForIDT(idt, "") << " length " << serial_size << std::endl;
  out.write((const char*) data, serial_size);
  out.flush();
  out.close();
  delete[] data;
  return true;
}

};