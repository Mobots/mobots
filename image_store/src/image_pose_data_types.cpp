#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <string>
#include <exception>
#include <iostream>

#include "image_pose_data_types.h"

/**
 * Loads the xml data from the filename
 */
void imagePoseData::load(const std::string &filename){
	// Create empty property tree object
	using boost::property_tree::ptree;
	ptree pt;
	// Load XML file and put its contents in property tree.
	read_xml(filename, pt);
	
	id.sessionID = pt.get("imageInfo.id.sessionID", 0);
	id.mobotID = pt.get("imageInfo.id.mobotID", 0);
	id.imageID = pt.get("imageInfo.id.imageID", 0);

	delPose.x = pt.get("imageInfo.delPose.x", 0);
	delPose.y = pt.get("imageInfo.delPose.y", 0);
	delPose.theta = pt.get("imageInfo.delPose.theta", 0);
	delPose.enable = pt.get("imageInfo.delPose.enable", false);
	
	absPose.x = pt.get("imageInfo.absPose.x", 0);
	absPose.y = pt.get("imageInfo.absPose.y", 0);
	absPose.theta = pt.get("imageInfo.absPose.theta", 0);
	absPose.enable = pt.get("imageInfo.absPose.enable", false);

	relPose.x = pt.get("imageInfo.relPose.x", 0);
	relPose.y = pt.get("imageInfo.relPose.y", 0);
	relPose.theta = pt.get("imageInfo.relPose.theta", 0);
	relPose.enable = pt.get("imageInfo.relPose.enable", false);

	image.width = pt.get("imageInfo.image.width", 0);
	image.height = pt.get("imageInfo.image.height", 0);
	image.encoding = pt.get<std::string>("imageInfo.image.encoding");
}

// Saves the data to the specified XML file
void imagePoseData::save(const std::string &filename_){
	// Create an empty property tree object
	using boost::property_tree::ptree;
	ptree pt;
	
	// Populate the property_tree with data
	pt.put("imageInfo.image.width", image.width);
	pt.put("imageInfo.image.height", image.height);
	pt.put("imageInfo.image.encoding", image.encoding);

	pt.put("imageInfo.id.sessionID", id.sessionID);
	pt.put("imageInfo.id.mobotID", id.mobotID);
	pt.put("imageInfo.id.imageID", id.imageID);
	
	pt.put("imageInfo.delPose.x", delPose.x);
	pt.put("imageInfo.delPose.y", delPose.y);
	pt.put("imageInfo.delPose.theta", delPose.theta);
	pt.put("imageInfo.delPose.enable", delPose.enable);

	pt.put("imageInfo.relPose.x", relPose.x);
	pt.put("imageInfo.relPose.y", relPose.y);
	pt.put("imageInfo.relPose.theta", relPose.theta);
	pt.put("imageInfo.relPose.enable", relPose.enable);
	
	pt.put("imageInfo.absPose.x", absPose.x);
	pt.put("imageInfo.absPose.y", absPose.y);
	pt.put("imageInfo.absPose.theta", absPose.theta);
	pt.put("imageInfo.absPose.enable", absPose.enable);
	
	// Write the property tree to the XML file.
	write_xml(filename_, pt);
}
