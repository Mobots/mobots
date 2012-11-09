#include <stdlib.h>
#include <sstream>
#include <sys/stat.h>
#include <mobots_common/utils.h>
#include <mobots_common/constants.h>


bool mobots_common::utils::parseNamespace(const std::string& nspace, int& mobotID){
  std::string mobot("mobot");
  size_t pos = nspace.find(mobot);
  if(pos == std::string::npos)
    return false;
  char c = nspace.at(pos+mobot.size());
  if(c == '0'){
    mobotID = 0;
    return true;
  }
  int number = atoi(nspace.substr(pos+mobot.size()).c_str());
  if(number == 0)
    return false;
  mobotID = number;
  return true;
}

static const char* HOME = getenv("HOME");
static std::string base = std::string(HOME)+std::string("/mobots-data");

static void replaceHome(std::string& path){
	std::string tilde("~");
	std::string home("$HOME");
	size_t start_pos = path.find("~");
	if(start_pos != std::string::npos){
		path.replace(start_pos, tilde.length(), HOME);
		return;
	}
	start_pos = path.find("$HOME");
	if(start_pos == std::string::npos)
		return;
	path.replace(start_pos, home.length(), HOME);
}

void mobots_common::store::setBasePath(std::string basePath){
	replaceHome(basePath);
	if(basePath.at(basePath.size()-1) == '/')
		base = basePath.substr(0, basePath.size()-1);
	else
		base = basePath;
}

std::string mobots_common::store::getPathForID(const int sessionID, const int mobotID, const int imageID, const std::string &fileEnding){
  std::stringstream ss;
  ss << base << "/session-" << sessionID << "/mobot-" << mobotID << "/" << imageID << fileEnding;
  return ss.str();
}

bool mobots_common::store::createDirs(int sessionID){
	for(int i = 0; i < mobots_common::constants::mobot_count; i++){
		std::stringstream stream;
		stream << "mkdir -p " << base << "/session-" << sessionID << "/mobot-" << i;
		system(stream.str().c_str());
		struct stat filestatus;
		std::stringstream ss;
		ss << base << "/session-" << sessionID << "/mobot-" << i;
		if(stat(ss.str().c_str(), &filestatus) == -1)
			return false;
	}
	return true;
}
