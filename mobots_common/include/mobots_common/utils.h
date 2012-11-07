#include <string>
#include <stdlib.h>

#pragma once

namespace mobots_common {
  namespace utils {
  
    /**
     * parses the mobotID from the given namespace 
     * e.g. "/mobot1/camera" results in 1
     * returns false if the namespace cannot be parsed
     */
    bool parseNamespace(const std::string& nspace, int& mobotID);
	};
	
	namespace store{
		/**
		 * Sets the base path (optional)
		 * default base is ~/mobots-data/
		 * remember to call createDirs after this call
		 */
		void setBasePath(std::string basePath);
		
    /**
     * returns the path for something to save
     */
    std::string getPathForID(const int sessionID, const int mobotID, const int imageID, const std::string& fileEnding);

		/**
		 * creates the dirs where images and features are saved for a given sessionID
		 * returns false on failure
		 */
    bool createDirs(const int sessionID);
  };
};
