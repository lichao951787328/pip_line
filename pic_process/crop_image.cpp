#include <opencv2/opencv.hpp>
#include <iostream>

// 全局变量
cv::Rect2i roi;
bool drawing = false;
cv::Mat image;
cv::Mat image_copy;

// 鼠标回调函数
void onMouse(int event, int x, int y, int, void*)
{
    if (event == cv::EVENT_LBUTTONDOWN) {
        // 开始绘制矩形
        drawing = true;
        roi = cv::Rect(x, y, 0, 0);
    }
    else if (event == cv::EVENT_MOUSEMOVE && drawing) {
        // 更新矩形
        roi.width = x - roi.x;
        roi.height = y - roi.y;
        image_copy = image.clone();
        cv::rectangle(image_copy, roi, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Image", image_copy);
    }
    else if (event == cv::EVENT_LBUTTONUP) {
        // 完成绘制
        drawing = false;
        if (roi.width < 0) {
            roi.x += roi.width;
            roi.width *= -1;
        }
        if (roi.height < 0) {
            roi.y += roi.height;
            roi.height *= -1;
        }
        image_copy = image.clone();
        cv::rectangle(image_copy, roi, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Image", image_copy);
    }
}

int main(int argc, char** argv)
{
    

    // 读取图像
    image = cv::imread("/home/lichao/TCDS/src/pip_line/pic_process/heightmap.png");
    if (image.empty()) {
        std::cerr << "Could not open or find the image" << std::endl;
        return -1;
    }

    // 创建窗口
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Image", onMouse);

    // 显示图像
    image_copy = image.clone();
    cv::imshow("Image", image_copy);

    // 等待用户按键退出
    int key = cv::waitKey(0);

    if (key == 's' && roi.width > 0 && roi.height > 0) {
        // 截取选定区域并保存
        cv::Mat croppedImage = image(roi);
        cv::imwrite("cropped_image.png", croppedImage);
        std::cout << "Cropped image saved as cropped_image.png" << std::endl;
    }

    return 0;
}
