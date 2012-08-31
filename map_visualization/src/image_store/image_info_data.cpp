#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <string>
#include <set>
#include <exception>
#include <iostream>

struct image_id{
	int sessionID;
	int mobotID;
	int imageID;
};

struct pose_t{
	float x;
	float y;
	float theta;
};

struct image_info_data{
	image_id id;
	std::set<pose_t> poses;
	std::string encoding;
	void load(const std::string &filename);
	void save(const std::string &filename);
};

void image_info_data::load(const std::string &filename){
	// Create empty property tree object
	using boost::property_tree::ptree;
	ptree pt;
	// Load XML file and put its contents in property tree.
	read_xml(filename, pt);
	
	encoding = pt.get<std::string>("image_info.encoding");
	
	id.sessionID = pt.get("image_info.id.sessionID", 0);

	//BOOST_FOREACH(ptree::value_type &v, pt.get_child("debug.poses"))
	    //poses.insert(v.second.data());
}

// Saves the debug_settings structure to the specified XML file
void image_info_data::save(const std::string &filename)
{
	// Create an empty property tree object
	using boost::property_tree::ptree;
	ptree pt;
	
	// Put log filename in property tree
	pt.put("image_info.encoding", encoding);
	
	// Put debug level in property tree
	pt.put("image_info.id.sessionID", id.sessionID);
	
	//BOOST_FOREACH(const std::string &name, m_modules)
	//pt.put("debug.modules.module", name, true);
	
	// Write the property tree to the XML file.
	write_xml(filename, pt);
}

int main()
{
	try
	{
		image_info_data iid;
		iid.load("image_info.xml");
		iid.save("image_info.xml");
		std::cout << "Success\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error: " << e.what() << "\n";
	}
	return 0;
} 
