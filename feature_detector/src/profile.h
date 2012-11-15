#pragma once

#if PROFILE
  #include <iostream>
  #include <opencv2/core/core.hpp>

  static double startTime;
  static std::string currentModule;

  inline void moduleStarted(const std::string moduleName){
    startTime = (double)cv::getTickCount();
    currentModule = moduleName;
  }
  inline void moduleEnded(){
    double execTime = ((double)cv::getTickCount() - startTime)/cv::getTickFrequency();
    std::cout << "[Profile] " << currentModule << ": " << execTime << "s" << std::endl;
  }
#else
  inline void moduleStarted(const std::string moduleName){}
  inline void moduleEnded(){}
#endif

