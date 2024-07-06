/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2024-06-07 11:01:20
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2024-06-08 14:50:26
 * @FilePath: /pip_line/src/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// 根据论文意见尝试更改,论文步骤:提取出平面,对非平面点构造高程图,剔除大于某一阈值的平面上所有点,使用使用其他点构造高程图,使用高程图进行落脚点规划,完成避障任务.

#include <ros/ros.h>
#include <iostream>
#include <pip_line/pip_line.h>
using namespace std;

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]); 
    google::InstallFailureSignalHandler();
    // google::SetCommandLineOptionWithMode("FLAGS_minloglevel", "2");
    FLAGS_minloglevel = 0;
    FLAGS_colorlogtostderr = true;
    FLAGS_alsologtostderr = true;
    ros::init(argc, argv, "pip_line");
    ros::NodeHandle n;
    pip_line ppl(n);

    ros::AsyncSpinner spinner(2);  // Use n threads
    spinner.start();
    ros::waitForShutdown();
    
    // ros::spin();
    return 0;
}