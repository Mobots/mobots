#include <string>
#include <stdlib.h>

#pragma once

namespace util{
  
/**
 * parses the mobotID from the given namespace 
 * e.g. "/mobot1/camera" results in 1
 * returns false if the namespace cannot be parsed
 */
bool parseNamespace(const std::string& nspace, int& mobotID){
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

};