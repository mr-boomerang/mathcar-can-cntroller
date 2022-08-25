#include <iostream>

#include <opencv2/opencv.hpp>

#include <utils/gui/imshow.hpp>

int main(int argc, char *argv[])
{

  if(argc < 2) {
    std::cout << "Provide image path as argument." << std::endl;
    return 1;
  }

  cv::Mat img = cv::imread(argv[1]);

  GuiCreate("Gl Window", WinSize::normal, 30);
  bool cont = true;
  //while (cont) {
    GuiInstance.show("Gl Window", img);
    cont = GuiInstance.waitKey();
    std::cout << "One iter done." << std::endl;
  //}

  return 0;
}
