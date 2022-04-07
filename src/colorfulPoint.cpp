// C++ 标准库
#include <iostream>
#include <string>
using namespace std;

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace cv;
 
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void Velodyne_To_PCD(string currFilenameBinary,string PCDDIR);

int main(int argc, char** argv)
{	
	string currFilenameBinary = "/home/lanpokn/Documents/2022/slam/ex1/04/velodyne/000000.bin";
	string PCDDIR = "/home/lanpokn/Documents/2022/slam/ex1/result/temp1.pcd";
	Velodyne_To_PCD(currFilenameBinary,PCDDIR);
	return 0;
}

void Velodyne_To_PCD(string currFilenameBinary,string PCDDIR)
{
	// allocate 4 MB buffer (only ~130*4*4 KB are needed)
	int32_t num = 1000000;
	float *data = (float*)malloc(num*sizeof(float));

	// pointers，TODO
	float *px = data+0;
	float *py = data+1;
	float *pz = data+2;
	//TODO
	float *pr = data+3;
	float pg = 0;
	float pb = 0;

	PointCloud point_cloud;
	// load point cloud
	FILE *stream;
	stream = fopen (currFilenameBinary.c_str(),"rb");
	num = fread(data,sizeof(float),num,stream)/4;
	for (int32_t i=0; i<num; i++) {
		PointT p;
		// 计算这个点的空间坐标
		p.z = *pz;
		p.x = *px;
		p.y = *py;

		// 从rgb图像中获取它的颜色
		// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
		// p.b = rgb.ptr<ushort>(m)[n*3];
		// p.g = rgb.ptr<ushort>(m)[n*3+1];
		p.r = *pr;//TODO, TOchange
		p.b = 0;
		p.g = 0;
		point_cloud.points.push_back(p);
		px+=4; py+=4; pz+=4; pr+=4;// it's plussing in the memory allocator
	}
	fclose(stream);
	free(data);

	point_cloud.height = 1;
	point_cloud.width = point_cloud.points.size();
	cout<<"point cloud size = "<<point_cloud.points.size()<<endl;
	point_cloud.is_dense = false;
	pcl::io::savePCDFile(PCDDIR, point_cloud);
}