//#include <ros/ros.h>
#include <string>

#include <stdlib.h>

#pragma once

namespace util{
  
/**
 * parses the namespace and returns the mobotID
 * e.g. "/mobot1/camera" returns 1
 * returns -1 if the namespace cannot be parsed
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