// C++ 标准库
#include <iostream>
#include <string>
using namespace std;

//eigen
#include<Eigen/Eigen>

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace cv;
 
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void Velodyne_To_PCD(string currFilenameBinary,string PCDDIR,string currFilenameImage);

// 相机内参
namespace{
const double camera_factor = 1;
const double camera_cx = 601.8873;
const double camera_cy = 183.1104;
const double camera_fx = 707.0912;
const double camera_fy = 721.5377;
Eigen::MatrixXd P2(3,4);
Eigen::MatrixXd Tr(4,4);
}
// calib.txt: Calibration data for the cameras: P0/P1 are the 3x4 projection
// matrices after rectification. Here P0 denotes the left and P1 denotes the
// right camera. Tr transforms a point from velodyne coordinates into the
// left rectified camera coordinate system. In order to map a point X from the
// velodyne scanner to a point x in the i'th image plane, you thus have to
// transform it like:

//   x = Pi * Tr * X

int main(int argc, char** argv)
{	
	// Mat picture = imread("1.jpg");
    // //图片必须添加到工程目下
    // //也就是和test.cpp文件放在一个文件夹下！！！
    // imshow("测试程序", picture);
    // waitKey(20150901);
	P2<<7.070912000000e+02,0.000000000000e+00,6.018873000000e+02,4.688783000000e+01,0.000000000000e+00,7.070912000000e+02,1.831104000000e+02,1.178601000000e-01,0.000000000000e+00,0.000000000000e+00,1.000000000000e+00,6.203223000000e-03;
	Tr<<-1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03, -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02, 9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03,-3.339968064433e-01,0,0,0,1;
	cout<<Tr(2,0)<<endl;
	string currFilenameBinary = "/home/lanpokn/Documents/2022/slam/ex1/04/velodyne/000000.bin";
	string PCDDIR = "/home/lanpokn/Documents/2022/slam/ex1/result/temp1.pcd";
	string currFilenameImage = "/home/lanpokn/Documents/2022/slam/ex1/04/image_2/000000.png";
	Velodyne_To_PCD(currFilenameBinary,PCDDIR,currFilenameImage);
	return 0;
}

void Velodyne_To_PCD(string currFilenameBinary,string PCDDIR,string currFilenameImage)
{	

	// allocate 4 MB buffer (only ~130*4*4 KB are needed)
	int32_t num = 1000000;
	float *data = (float*)malloc(num*sizeof(float));

	// pointers，TODO
	float *px = data+0;
	float *py = data+1;
	float *pz = data+2;
	Eigen::MatrixXd x_velodyne(4,1);
	Eigen::MatrixXd x_P2(3,1);
	Eigen::MatrixXd x_P2_xyz(4,1);
	//TODO
	float *pr = data+3;
	float pg = 0;
	float pb = 0;

	PointCloud point_cloud;
	// load point cloud
	FILE *stream;
	stream = fopen (currFilenameBinary.c_str(),"rb");
	num = fread(data,sizeof(float),num,stream)/4;

	//OPEN IMAGE
	cv::Mat rgb;
	rgb = cv::imread(currFilenameImage);
	int m_max = 0;
	int n_max = 0;
	for (int32_t i=0; i<num; i++) {
		PointT p;
		// 计算这个点的空间坐标
		p.z = *pz;
		p.x = *px;
		p.y = *py;

		// 从rgb图像中获取它的颜色
		x_velodyne<<(*px),(*py),(*pz),1;
		//TODO out of range , so the change is wrong
		// how to calculate the right m and n
		//may be there need a transfer from cm to m?,ask!
		x_P2_xyz = Tr * x_velodyne;
		double xyz[4] = {x_P2_xyz (0),x_P2_xyz(1),x_P2_xyz(2),x_P2_xyz(3)};
		x_P2 = P2 * Tr * x_velodyne;
		cout<<"xyz="<<xyz[0]<<" "<<xyz[1]<<" "<<xyz[2]<<" "<<xyz[3]<<endl;
		cout<<"x_P2 ="<<x_P2<<endl;
		// normalization
		int n = x_P2(0)/x_P2(2);
		int m = x_P2(1)/x_P2(2);
		if (m>m_max){
			m_max = m;
		}
		if(n>n_max){
			n_max = n;
		}
		int one = x_P2(2);
		// 
		// if(m<rgb.rows && 3*n+2<rgb.cols && m>=0 &&n>=0){
		// m is column and n is rows
		if(m<rgb.rows and 3*n+2<rgb.cols and x_P2(2)>0 and m>0 and n>0){
			if(m<0){
				printf("error");
			}//bgr
			p.r = rgb.ptr<ushort>(m)[3*n+2];
			p.b = rgb.ptr<ushort>(m)[n*3];
			p.g = rgb.ptr<ushort>(m)[n*3+1];
		}
		else{
			p.r = 100;
			p.b = 0;
			p.g = 0;
		}
		point_cloud.points.push_back(p);
		px+=4; py+=4; pz+=4; pr+=4;// it's plussing in the memory allocator
	}
	cout<<x_P2;
	fclose(stream);
	free(data);

	point_cloud.height = 1;
	point_cloud.width = point_cloud.points.size();
	cout<<"point cloud size = "<<point_cloud.points.size()<<endl;
	point_cloud.is_dense = false;
	pcl::io::savePCDFile(PCDDIR, point_cloud);
}