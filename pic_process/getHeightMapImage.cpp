#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
// 鼠标回调函数
void onMouse(int event, int x, int y, int, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN) {
        cv::Mat* image = reinterpret_cast<cv::Mat*>(userdata);
        if (image->channels() == 3) {
            cv::Vec3b color = image->at<cv::Vec3b>(cv::Point(x, y));
            std::cout << "Position: (" << x << ", " << y << ") - BGR Color: ("
                      << (int)color[0] << ", " << (int)color[1] << ", " << (int)color[2] << ")" << std::endl;
        } else if (image->channels() == 1) {
            uchar color = image->at<uchar>(cv::Point(x, y));
            std::cout << "Position: (" << x << ", " << y << ") - Gray Color: "
                      << (int)color << std::endl;
        }
    }
}

int main(int argc, char** argv)
{
    // 检查是否提供了图像路径
    // if (argc < 2) {
    //     std::cerr << "Usage: " << argv[0] << " <image_path>" << std::endl;
    //     return -1;
    // }

    // 读取图像
    cv::Mat image = cv::imread("/home/lichao/TCDS/src/pip_line/pic_process/heightmap2.png");
    if (image.empty()) {
        std::cerr << "Could not open or find the image" << std::endl;
        return -1;
    }
    cout<<image.size()<<endl;
    cv::Rect roi(69, 10, 930, 840);
    cv::Mat croppedImage = image(roi);
    int targetWidth = 800;
    double scale = static_cast<double>(targetWidth) / 930;
    int targetHeight = static_cast<int>(840 * scale);
    cv::Mat resizedImage;
    cv::resize(croppedImage, resizedImage, cv::Size(targetWidth, targetHeight));

    cv::Mat finalImage = cv::Mat::zeros(800, 800, resizedImage.type());
    // 计算复制到finalImage的起始位置
    int startX = (finalImage.cols - resizedImage.cols) / 2;
    int startY = (finalImage.rows - resizedImage.rows) / 2;

    // 复制resizedImage到finalImage中心位置
    resizedImage.copyTo(finalImage(cv::Rect(startX, startY, resizedImage.cols, resizedImage.rows)));

    for (int i = 0; i < finalImage.cols; i++)
    {
        for (int j = 0; j < finalImage.rows; j++)
        {
            if (finalImage.at<cv::Vec3b>(i, j) == cv::Vec3b(48, 48, 48))
            {
                finalImage.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    cv::Mat rotatedImage;
    cv::rotate(finalImage, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);
    
    // // 将彩色图像转换为灰度图像
    // cv::Mat grayImage;
    // cv::cvtColor(finalImage, grayImage, cv::COLOR_BGR2GRAY);

    // // 设定阈值，将灰度图像中接近黑色的部分变成纯黑
    // int thresholdValue = 40;  // 可调整的阈值
    // cv::Mat binaryImage;
    // cv::threshold(grayImage, binaryImage, thresholdValue, 255, cv::THRESH_BINARY);

    // // 将二值图像应用到原始彩色图像上
    // cv::Mat resultImage;
    // finalImage.copyTo(resultImage, binaryImage);

    cv::imshow("Original Image", image);
    cv::imshow("Cropped Image", croppedImage);
    cv::imshow("Resized Image", resizedImage);
    cv::imshow("Final Image", finalImage);
    cv::imshow("rotatedImage", rotatedImage);
    cv::imwrite("finalImage.png", finalImage);
    cv::imwrite("rotatedImage.png", rotatedImage);
    // cv::imshow("resultImage ", resultImage);
    // 创建窗口
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

    // 设置鼠标回调函数
    cv::setMouseCallback("Image", onMouse, &image);

    // 显示图像
    cv::imshow("Image", image);

    // 等待用户按键退出
    cv::waitKey(0);

    return 0;
}
