#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <string>
#include <set>
#include <exception>
#include <iostream>

struct mobot_id_t{
	int sessionID;
	int mobotID;
	int imageID;
};

struct pose_t{
	float x;
	float y;
	float theta;
	bool enable;
};

struct image_info_data{
	mobot_id_t id;
	pose_t rel_pose; // relative_pose
	pose_t abs_pose; // absolute_pose
	std::string encoding;
	void load(const std::string &filename);
	void save(const std::string &filename, const std::string &encoding_, const mobot_id_t *id, const pose_t *rel_pose_, const pose_t *abs_pose_);
};

void image_info_data::load(const std::string &filename){
	// Create empty property tree object
	using boost::property_tree::ptree;
	ptree pt;
	// Load XML file and put its contents in property tree.
	read_xml(filename, pt);
	
	id.sessionID = pt.get("image_info.id.sessionID", 0);
	id.mobotID = pt.get("image_info.id.mobotID", 0);
	id.imageID = pt.get("image_info.id.imageID", 0);

	abs_pose.x = pt.get("image_info.abs_pose.x", 0);
	abs_pose.y = pt.get("image_info.abs_pose.y", 0);
	abs_pose.theta = pt.get("image_info.abs_pose.theta", 0);
	abs_pose.enable = pt.get("image_info.abs_pose.enable", false);

	rel_pose.x = pt.get("image_info.rel_pose.x", 0);
	rel_pose.y = pt.get("image_info.rel_pose.y", 0);
	rel_pose.theta = pt.get("image_info.rel_pose.theta", 0);
	rel_pose.enable = pt.get("image_info.rel_pose.enable", false);

	encoding = pt.get<std::string>("image_info.encoding");
}

// Saves the debug_settings structure to the specified XML file
void image_info_data::save(const std::string &filename_, const std::string &encoding_, const mobot_id_t *id_, const pose_t *rel_pose_, const pose_t *abs_pose_)
{
	// Create an empty property tree object
	using boost::property_tree::ptree;
	ptree pt;
	
	// Put log filename in property tree
	pt.put("image_info.encoding", encoding_);
	
	// Put debug level in property tree
	pt.put("image_info.id.sessionID", id_->sessionID);
	pt.put("image_info.id.mobotID", id_->mobotID);
	pt.put("image_info.id.imageID", id_->imageID);
	
	pt.put("image_info.rel_pose.x", rel_pose_->x);
	pt.put("image_info.rel_pose.y", rel_pose_->y);
	pt.put("image_info.rel_pose.theta", rel_pose_->theta);
	pt.put("image_info.rel_pose.enable", rel_pose_->enable);
	
	pt.put("image_info.abs_pose.x", abs_pose_->x);
	pt.put("image_info.abs_pose.y", abs_pose_->y);
	pt.put("image_info.abs_pose.theta", abs_pose_->theta);
	pt.put("image_info.abs_pose.enable", abs_pose_->enable);
	
	// Write the property tree to the XML file.
	write_xml(filename_, pt);
}

int main()
{
	try
	{
		const mobot_id_t id{1, 3, 6};
		const pose_t rel_pose{12.1, 23.4, 78.2, 0};
		const pose_t abs_pose{56.1, 25.4, 8.2, true};
		image_info_data iid;
		iid.save("image_info.xml", "jpg", &id, &rel_pose, &abs_pose);
		iid.load("image_info.xml");
		std::cout << "Success\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error: " << e.what() << "\n";
	}
	return 0;
} 
