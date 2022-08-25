#ifdef UTILS_USE_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

#include <utils/color.hpp>
#include <iostream>

#ifdef UTILS_USE_OPENCV
cv::Scalar demoFloatOperator(float *col) {
  return cv::Scalar(col[2]*255, col[1]*255, col[0]*255, col[3]*255);
}
#else
void demoFloatOperator(float *col) {
  std::cout << "col = " <<col[0] << ", " << col[1] << ", " << col[2]
  << ", " << col[3] << std::endl;
}

#endif

void demoColArgument(utils::Color col) {
  std::cout << "Got Col" << std::endl;
}


using namespace utils;

int main(int argc, char *argv[])
{
  Color color_1 = Color::hgreen;
  color_1 += Color::red;

  Color color_2 = Color::blue + Color::cyan;

#ifdef UTILS_USE_OPENCV
  cv::Mat img = cv::Mat::zeros(300, 300, CV_8UC3);
  cv::circle(img, cv::Point(50, 50), 50, color_1, -1);
  cv::circle(img, cv::Point(50, 150), 50, color_2, -1);
  cv::circle(img, cv::Point(50, 250), 50, Color(Color::red), -1);
  cv::circle(img, cv::Point(150, 50), 50, demoFloatOperator(color_2), -1);

  cv::imshow("out", img);
  cv::waitKey();
#else
  demoFloatOperator(color_1);
  demoFloatOperator(color_2);
#endif
  demoColArgument(Color::blue);

  return 0;
}
