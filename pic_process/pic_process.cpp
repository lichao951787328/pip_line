#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 读取图像
    cv::Mat img = cv::imread("/home/lichao/TCDS/src/pip_line/pic_process/dilated_image_upper0.png");
    if (img.empty()) {
        std::cout << "Could not open or find the image!" << std::endl;
        return -1;
    }

    // 创建一个输出图像
    cv::Mat output = img.clone();

    // 遍历图像的每个像素
    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {
            // 获取像素值
            cv::Vec3b color = img.at<cv::Vec3b>(y, x);

            // 检查是否为黑色（假设黑色为RGB(0,0,0)）
            if (color[0] == 0 && color[1] == 0 && color[2] == 0) {
                // 将黑色部分变为白色（RGB(255,255,255)）
                output.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
            }
            // 检查是否为白色（假设白色为RGB(255,255,255)）
            else if (color[0] == 255 && color[1] == 255 && color[2] == 255) {
                // 将白色部分变为红色（RGB(0,0,255)）51, 160, 44
                output.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
            }
        }
    }

    // 显示原图和处理后的图像
    cv::imshow("Original Image", img);
    cv::imshow("Processed Image", output);

    // 保存处理后的图像
    cv::imwrite("/home/lichao/TCDS/src/pip_line/pic_process/processed_image.jpg", output);

    // 等待按键按下
    cv::waitKey(0);

    return 0;
}
