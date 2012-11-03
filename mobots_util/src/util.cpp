#include <string>
#include <stdlib.h>
#include <mobots_util/util.h>


bool mobots_util::parseNamespace(const std::string& nspace, int& mobotID){
  std::string mobot("mobot");
  int pos = nspace.find(mobot);
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

std::string mobots_util::image_store::getPathForID(const int sessionID, const int mobotID, const int imageID, const char* fileEnding){
  std::stringstream ss;
  ss << base << "/session-" << sessionID << "/mobot-" << mobotID << "/" << imageID << fileEnding;
  return ss.str();
}