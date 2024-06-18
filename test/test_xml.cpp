/*
 * @description: 
 * @param : 
 * @return: 
 */

#include <iostream>
#include "plane_detection/type.h"
#include "plane_detection/load_parameter.h"
int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]); 
  google::InstallFailureSignalHandler();
  FLAGS_minloglevel = 0;
  FLAGS_colorlogtostderr = true; 
  FLAGS_alsologtostderr = true;
  std::cout<<"....."<<std::endl;
  parameter p;
  if (load_parameter(p) == 0)
  {
    p.showParameter();
  }
  google::ShutdownGoogleLogging();
  return 0;
}