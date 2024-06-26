# 实验二.视觉 3D 点云图构建与深度测量
## 一.环境具体过程

### (1) 实验问题和解决方法

![image-20240605175344093](C:\Users\14811\AppData\Roaming\Typora\typora-user-images\image-20240605175344093.png)



### (2) 实验结果

![image-20240607165123470](C:\Users\14811\AppData\Roaming\Typora\typora-user-images\image-20240607165123470.png)





## 二.实验代码分析
### (1) 实验源码
```c++
#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>  
#include <boost/format.hpp>  
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main( int argc, char** argv)
{
    vector<cv::Mat> colorImgs, depthImgs;
    vector<Eigen::Isometry3d,
           Eigen::aligned_allocator<Eigen::Isometry3d>> poses; // 相机位姿
    ifstream fin("./pose.txt");
    if (!fin)
    {
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
        return 1;
    }
    for ( int i=0; i<5; i++ )
    {
        boost::format fmt( "./%s/%d.%s" ); //图像文件格式
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); 
       
        double data[7] = {0};
        for ( auto& d:data ) fin>>d;

        /*Quaterniond代表四元数，data[6]是实数，把实数放在第一位;
          Isometry3代表变换矩阵，变换矩阵初始化旋转矩阵部分;
          Vector3d代表平移向量，变换矩阵初始化平移向量部分*/
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d T(q); 
        T.pretranslate( Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back( T );
    }

    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;

    cout<<"正在将图像转换为点云..."<<endl;

    /*定义点云使用的格式：这里用的是XYZRGB*/
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    PointCloud::Ptr pointCloud( new PointCloud );//新建一个点云

    /*将5张图片的像素坐标转换到相机坐标，再转换到世界坐标存储到点云格式的变量中，
      for循环之后用pcl的相关函数将点云转换到pcl能够显示的格式*/
    for ( int i=0; i<5; i++ )
    {
        cout<<"转换图像中: "<<i+1<<endl;
        cv::Mat color = colorImgs[i];  //颜色信息
        cv::Mat depth = depthImgs[i];  //深度信息
        Eigen::Isometry3d T = poses[i];  //相机位姿，即外参

        /*对图像像素进行坐标转换，将图像的坐标通过内参矩阵K转换为相机坐标系下的坐标，
          之后通过外参矩阵T,转化为世界坐标系下的坐标*/
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                if ( d==0 ) continue; // 为0表示没有测量到
                Eigen::Vector3d point;
                point[2] = double(d)/depthScale;  //对实际尺度的一个缩放
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy;
                Eigen::Vector3d pointWorld = T*point; //将相机坐标系转换为世界坐标系

                PointT p ;
                p.x = pointWorld[0];//将世界坐标系下的坐标用pcl专门的点云格式存储起来
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];
                pointCloud->points.push_back( p );
            }
    }

    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );
    return 0;
}
```
### (2) 代码分析
#### 1.预备知识
##### <1> C++中的vector用法
**1.基本操作**

1)添加头文件

```
#include<vector>；
```

2)创建vector对象,vec为对象

```c++
vector<int> vec
```

3)尾部插入数字

```c++
vec.push_back(a)；
```

4)使用下标访问元素，下标是从0开始的

```c++
cout<<vec[0]<<endl;
```

5)使用迭代器访问元素

```c++
vector<int>::iterator it;
for(it=vec.begin();it!=vec.end();it++)
    cout<<*it<<endl;
```

6)插入元素,在第i+1个元素前面插入a

```c++
vec.insert(vec.begin()+i,a);
```

7)删除元素

```c++
vec.erase(vec.begin()+2); //删除第3个元素
vec.erase(vec.begin()+i,vec.end()+j); //删除区间[i,j-1];区间从0开始
```
8)向量的大小

```c++
vec.size();
```

9)清空vector

```c++
vec.clear();
```

---

**2.其他知识**

1) vector的元素不仅仅可以使int,double,string,还可以是结构体，但是要注意：结构体要定义为全局的，否则会出错。

2) 使用reverse将元素翻转

```c++
#include<algorithm>
reverse(vec.begin(),vec.end());  //将元素翻转(在vector中，如果一个函数中需要两个迭代器，一般后一个都不包含.)
```

3) 使用sort排序

```c++
#include<algorithm>，
sort(vec.begin(),vec.end());  //(默认是按升序排列,即从小到大)
```

---
##### <2> Eigen库中各种形式的表示
|相关表示方法|Eigen相关类说明|
|-------------|---------------|
|旋转矩阵（3X3)|Eigen::Matrix3d|
|旋转向量（3X1）|Eigen::AngleAxisd|
|四元数（4X1）|Eigen::Quaterniond|
|平移向量（3X1）|Eigen::Vector3d|
|变换矩阵（4X4）|Eigen::Isometry3d|
---
##### <3> 具体函数的解释
**format的使用说明**

```c++
boost::format fmt( "./%s/%d.%s" );
colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 ));
```
> 这里我们只分析format的用法，imread下面会进行解释。
boost::format类提供了类似C语言里'printf'功能的格式化输出能力，格式也很类似。
首先我们先类对比一下format的一般形式和printf的相似处,
其中%2.3 对应 %1.23456，%d 对应 %12，代码如下：
```c++
cout << boost::format("%2.3f, %d")  %1.23456  %12;
printf( "%2.3f, %d\n" , 1.23456, 12 );
/*两条语句输出结果都是1.234,12*/
```
> 然后我们来看一下boost::format对象的操作，代码如下：
```c++
// 对象的形式，%X%为占位符
boost::format fmt("%2% \n %1% \n %3%" );
fmt %"first";  //对应第二个元素%1%
fmt %"second";  // 对应第一个元素%2%
fmt %"third";  //对应第三个元素%3%
string s=fmt.str();  //转成字符串的形式
string s2=boost::str(fmt);
```
> 最后总结一下format的三种方式，代码如下：
```c++
// 方式1
cout<<boost::format("%s") % "输出内容" <<endl;
// 方式2
boost::format fmt("%s");
fmt % "输出内容"；
string s=fmt.str();
cout<<s<<endl;
// 方式3
string s;
s=str(boost::format("%s") % "输出内容")；
cout<<s <<endl;
```
> 很显然，本实验用的是**方式2**,
第一个占位符%s对应的是color还是depth文件夹；
第二个占位符%d对应的是i+1，也就是图片的名字1-5；
第三个占位符%s对应的是文件名的后缀png或pgm，
通过这样的方式就可以拼接出两个文件夹的图片文件名，
最后通过cv::imread把图片读取出来。
---
**imread函数说明**

```c++
colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 ));
```
> imread的函数原型是：Mat imread( const string& filename, int flags=1 );
> Mat是OpenCV里的一个数据结构，在这里定义一个Mat类型的变量img，用于保存读入的图像，用imread函数来读取图像，
> 第一个字段标识图像的文件名（包括扩展名），第二个字段用于指定读入图像的颜色和深度，它的取值可以有以下几种：
>
> 1.CV_LOAD_IMAGE_UNCHANGED (<0)，以原始图像读取（包括alpha通道）；
>
> 2.CV_LOAD_IMAGE_GRAYSCALE ( 0)，以灰度图像读取；
>
> 3.CV_LOAD_IMAGE_COLOR (>0)，以RGB格式读取
>
> 第二个参数默认是1，所以这里colorImgs读取RGB图像，depthImgs读取原始图像
---
**push_back方法介绍**
```c++
colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 ));
```
> vector::void push_back (const value_type& val);
vector::void push_back (value_type&& val);
该函数将一个新的元素加到vector的最后面，位置为当前最后一个元素的下一个元素，
新的元素的值是val的拷贝（或者是移动拷贝）。
在本实验中，通过push_back方法把所有图片都读取到colorImgs和depthImgs中。

## 三.理论知识总结
### (1) 代码框架
* 首先，根据颜色图像信息、深度图像信息和相机内参，我们可以计算出像素点在相机坐标系下的位置，
* 然后，根据相机位姿信息，我们可以计算这些像素在世界坐标系下的位置。
* 最后，求出所有像素的世界坐标信息（点云），并全部加起来可得到一张类似于地图的东西。
### (2) 理论知识

拍照的过程：世界坐标系——相机坐标系——成像平面坐标系——像素坐标系

计算点云的过程：像素坐标系——成像平面坐标系——相机坐标系——世界坐标系

#### 1.单目相机
##### <1> 单目相机的内参
1.表示形式：矩阵

2.推导过程：

![](.\pictures\one_inside.jpg)

3.代码实现：

```c++
/*厂家给定的*/
double cx = 325.5;
double cy = 253.5;
double fx = 518.0;
double fy = 519.0;
```
---
##### <2> 单目相机的外参
1.表示形式：矩阵

2.推导过程：

![](.\pictures\one_outside.jpg)

3.代码实现：

```c++
for ( int i=0; i<5; i++ )
{
   boost::format fmt( "./%s/%d.%s" ); //图像文件格式
   colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
   depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 使用-1读取原始图像
   double data[7] = {0};
   for ( auto& d:data ) fin>>d;

   /*Quaterniond代表四元数，data[6]是实数，把实数放在第一位*/
   Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );

   /*Isometry3代表变换矩阵，变换矩阵初始化旋转矩阵部分*/
   Eigen::Isometry3d T(q);
   /*Vector3d代表平移向量，变换矩阵初始化平移向量部分*/
   T.pretranslate( Eigen::Vector3d(data[0], data[1], data[2]));

   /*把变换矩阵存储到位姿数组中，poses代表相机外参*/
   poses.push_back( T );
}

for ( int i=0; i<5; i++ )
{
  Eigen::Isometry3d T = poses[i];  //相机位姿，即外参
}
```
---
#### 2.双目相机

##### <1> 双目相机的原理

![](.\pictures\two_camera.jpg)

##### <2> 双目相机的内参

内参包括相机的焦距f，成像原点cx,cy

---
##### <3> 双目相机的外参

外参包括两个摄像头之间的相对位置（即右摄像头相对于左摄像头的旋转矩阵R、平移向量t），就是本征矩阵

---

##### <4>双面相机的本征矩阵

本征矩阵E是反映【**空间一点P的像点】**在【**不同视角摄像机】**下【**相机坐标系】**中的表示之间的关系。

E = t^R，其中t^是**反对称矩阵(3 * 3)**,R是**旋转矩阵(3 * 3)**,最后的本征矩阵也是3 * 3
