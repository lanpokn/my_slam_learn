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


// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
// camera.fx=721.5377000000
// camera.fy=721.5377000000
// camera.cx=609.5593000000
// camera.cy=172.8540000000
// camera.baseline=0.53715
// camera.scale=10000
// 相机内参
const double camera_factor = 1;
const double camera_cx = 609.5593;
const double camera_cy = 172.854;
const double camera_fx = 721.5377;
const double camera_fy = 721.5377;

// 主函数
int main( int argc, char** argv )
{
    // 读取./data/rgb.png和./data/depth.png，并转化为点云

    // 图像矩阵
    int startIndex = 0,endIndex = 801;
    for (int index = startIndex;index < endIndex;index++)//从起始序列循环至截止序列
    {       
        //gray 相当于只有一个通道的rgb，我用了蓝色，因为点云看着顺眼
        string rgbDir = "../kitti03/image_0/";//获取视图输入目录名
        string depthDir = "../kitti03/depth_16/";
        string pointDir = "../kitti03/pointcloud16/";
        string rgbExt = ".png";
        //输出当前文件序号（使用的TUM数据集，其双目视图命名从000000至004540，详情参看博文末尾ps）
        string numzero;
        if ( index >= 0 && index <= 9 )
        {
            numzero = "00000";
        }
        else if ( index >= 10 && index <= 99 )
        {
            numzero = "0000";
        }
        else if ( index >= 100 && index <= 999 )
        {
            numzero = "000";
        }
        else if ( index >= 1000 && index <= 9999 )
        {
            numzero = "00";
        }
        else if ( index >= 10000 && index <= 99999 )
        {
            numzero = "0";
        }
        //get gray image
        cv::Mat rgb, depth;

        stringstream ss;
        ss<<rgbDir<<numzero<<index<<rgbExt;
        string filename;
        ss>>filename;
        rgb = cv::imread(filename);
        // rgb 图像是8UC3的彩色图像
        // depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
        ss.clear();
        filename.clear();
        ss<<depthDir<<numzero<<index<<rgbExt;
        ss>>filename;

        depth = cv::imread(filename,-1);
        cv::namedWindow("depth");
        cv::imshow("depth", depth);
        if ((cv::waitKey() & 255) == 27)
            break;
        // 点云变量
        // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
        PointCloud::Ptr cloud ( new PointCloud );
        // 遍历深度图
        for (int m = 0; m < depth.rows; m++)
            for (int n=0; n < depth.cols; n++)
            {
                // 获取深度图中(m,n)处的值
                ushort d = depth.ptr<ushort>(m)[n];
                // d 可能没有值，若如此，跳过此点
                if (d == 0)
                    continue;
                // d 存在值，则向点云增加一个点
                PointT p;

                // 计算这个点的空间坐标
                p.z = double(d) / camera_factor;
                p.x = (n - camera_cx) * p.z / camera_fx;
                p.y = (m - camera_cy) * p.z / camera_fy;

                // 从rgb图像中获取它的颜色
                // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
                // p.b = rgb.ptr<ushort>(m)[n*3];
                // p.g = rgb.ptr<ushort>(m)[n*3+1];
                // p.r = rgb.ptr<ushort>(m)[n*3+2];
                p.b = rgb.ptr<uchar>(m)[n];
                p.g = 0;
                p.r = 0;
                cloud->points.push_back( p );
                // 把p加入到点云中
            }
        // 设置并保存点云
        cloud->height = 1;
        cloud->width = cloud->points.size();
        cout<<"point cloud size = "<<cloud->points.size()<<endl;
        cloud->is_dense = false;

        ss.clear();
        filename.clear();
        ss<<pointDir<<numzero<<index<<".pcd";
        ss>>filename;
        pcl::io::savePCDFile(filename, *cloud );
        // 清除数据并退出
        cloud->points.clear();
        cout<<"Point cloud saved."<<endl;
        cout<<index<<endl;
    }
    return 0;
}