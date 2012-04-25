#pragma once

#define PROFILE 1

#ifdef PROFILE
  #include <iostream>
  #include <opencv2/core/core.hpp>

  static double startTime;
  static string currentModule;

  inline void moduleStarted(const string moduleName){
    startTime = (double)getTickCount();
    currentModule = moduleName;
  }
  inline void moduleEnded(){
    double execTime = ((double)getTickCount() - startTime)/getTickFrequency();
    cout << "[Profile] " << currentModule << ": " << execTime << "s" << endl;
  }
#else
  inline void moduleStarted(const string moduleName){}
  inline void moduleEnded(){}
#endif

