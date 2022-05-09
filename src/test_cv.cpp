// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

int main(int argc, char** argv)
{	
	Mat picture = imread("/home/lanpokn/Documents/2022/slam/ex1/04/image_2/000000.png");
    //图片必须添加到工程目下
    //也就是和test.cpp文件放在一个文件夹下！！！
    imshow("测试程序", picture);
    waitKey(20220405);
}