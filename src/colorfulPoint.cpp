
#include <iostream>  
#include<opencv2/opencv.hpp>
 
using namespace std;
using namespace cv;
 
int main(int argc, char** argv)
{
	Mat src;//初始化一个操作对象
	src = imread("/home/lanpokn/Documents/2022/slam/ex1/04/image_2/000000.png");
	if (!src.data)//判断图片是否加载进来
	{
		cout << "不能加载图片" << endl;
		return -1;
	}
	namedWindow("加载的图片", WINDOW_AUTOSIZE);
	imshow("加载的图片", src);//""内命名一致，才能显示在一个窗口
	waitKey(0);
	return 0;
}