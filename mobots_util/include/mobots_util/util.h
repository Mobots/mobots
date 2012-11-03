#include <string>
#include <stdlib.h>

#pragma once

namespace mobots_util{
  
/**
 * parses the mobotID from the given namespace 
 * e.g. "/mobot1/camera" results in 1
 * returns false if the namespace cannot be parsed
 */
bool parseNamespace(const std::string& nspace, int& mobotID);

namespace image_store{
	/**
	 * returns the path for something to save
	 */
	std::string getPathForID(const int sessionID, const int mobotID, const int imageID, const char* fileEnding);
	bool createDirs(const int sessionID);
}

};