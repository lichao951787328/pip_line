/*
 * @description: 
 * @param : 
 * @return: 
 */
/*
 * @description: 
 * @param : 
 * @return: 
 */

#ifndef _LOAD_PARAMETER_H_
#define _LOAD_PARAMETER_H_
#include <iomanip>
#include "tinyxml2.h"
#include "plane_detection/static_type.h"
#include <string>
#include <unistd.h>
#include <filesystem>
using namespace std;
std::string GetCurrentWorkingDir()
{
  std::string cwd("\0",FILENAME_MAX+1);
  return getcwd(&cwd[0],cwd.capacity());
}

int load_parameter(parameter & p, string path)
{
  // std::filesystem::path cwdd = std::filesystem::current_path();
  // std::cout<<cwdd.string()<<endl;
  char amountText[100];
  tinyxml2::XMLDocument doc;
  // std::string path = GetCurrentWorkingDir() + "../../config/accounts.xml";
  // std::cout<<path<<std::endl;
  // int res = doc.LoadFile((char*)path.data());
  // string package_name = "plane_detection";
  // std::string package_path = ros::package::getPath(package_name);

  // // 检查是否成功找到包路径
  // if (package_path.empty()) {
  //     std::cerr << "Error: Could not find package " << package_name << std::endl;
  // }
  int res = doc.LoadFile(path.c_str());

  if (res != 0)
  {
    std::cout<<"load xml file failed"<<std::endl;
    return res;
  }
  tinyxml2::XMLElement * pRootElement = doc.RootElement();

  if (NULL != pRootElement) 
  {
    tinyxml2::XMLElement * pParameters = pRootElement->FirstChildElement("parameters");
    if (NULL != pParameters) 
    {
      tinyxml2::XMLElement * pleafnode_width = pParameters->FirstChildElement("leafnode_width");
      if (1 != sscanf(pleafnode_width->GetText(), "%zu", &p.leafnode_width))
      {
        return -1;
      }
      
      tinyxml2::XMLElement * ppatch_num_percent_th = pParameters->FirstChildElement("patch_num__percent_th");
      p.patch_num_percent_th = atof(ppatch_num_percent_th->GetText());

      tinyxml2::XMLElement * ppatch_mse_th = pParameters->FirstChildElement("patch_mse_th");
      p.patch_mse_th = atof(ppatch_mse_th->GetText());

      tinyxml2::XMLElement * peigen_value_th = pParameters->FirstChildElement("eigen_value_th");
      p.eigen_value_th = atof(peigen_value_th->GetText());

      tinyxml2::XMLElement * pmerge_normal_dot_th = pParameters->FirstChildElement("merge_normal_dot_th");
      p.merge_normal_dot_th = atof(pmerge_normal_dot_th->GetText());

      tinyxml2::XMLElement * pmerge_normal_distance = pParameters->FirstChildElement("merge_normal_distance");
      p.merge_normal_distance = atof(pmerge_normal_distance->GetText());

      tinyxml2::XMLElement * pquatree_merge_normal_dot_th = pParameters->FirstChildElement("quatree_merge_normal_dot_th");
      p.quatree_merge_normal_dot_th = atof(pquatree_merge_normal_dot_th->GetText());
    }
  }
  return 0;
}

#endif
