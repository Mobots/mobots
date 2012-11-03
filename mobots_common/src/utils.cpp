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
static std::string base = std::string(HOME)+std::string("mobots-data");

std::string mobots_common::utils::getPathForID(const int sessionID, const int mobotID, const int imageID, const char* fileEnding){
  std::stringstream ss;
  ss << base << "/session-" << sessionID << "/mobot-" << mobotID << "/" << imageID << fileEnding;
  return ss.str();
}

bool mobots_common::utils::createDirs(int sessionID){
	for(int i = 0; i < 3; i++){
		std::stringstream stream;
		stream << "mkdir $HOME/mobots-data/session-" << sessionID << "/mobot-" << i;
		system(stream.str().c_str());
		struct stat filestatus;
		if(stat(stream.str().c_str(), &filestatus) == -1)
			return false;
	}
	return true;
}
