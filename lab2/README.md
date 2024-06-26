# Lab2环境搭建指南



### 0. 弄清楚实验背景

``` C++
  这个实验的主要目的是需要搭建好PCL库环境，同时能够利用PCL库和Eigen库实现实验指导手册中需要完成的"点云地图拼接"和"双目视觉系统标定与深度测量"，但我们在这里不做，你需要根据这个README文档搭建出对应的环境，能够在老师面前跑通我的测试文件以及实验指导手册中的其中一个内容: 点云地图拼接。具体的实验课注意事项请看文末.
  下面我来简要讲解一下实验涉及到的重点知识点:
  ① PCL(Point Cloud Library),点云库，点云是一系列三维坐标的离散点集，用于在图像处理任务中描述三维空间的物体坐标/轮廓/边界 等信息，而点云库就是一个开源的C++编写的点云处理库，提供了大量的接口来操作点云数据
  ② Eigen:  加速线性代数处理库，一样是C++编写的，PCL库中的数学运算实现依赖于Eigen库，所以需要先搭建好Eigen库，然后才能搭建PCL库环境
```





### 1. 搭建PCL库环境

``` C++
1. 注意:  指导PDF中的安装PCL指导有问题，按如下步骤安装:
① sudo apt update   ② sudo apt install libpcl-dev
③ 用测试文件进行测试  
/**  test.cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (auto& point : cloud.points) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    std::cout << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

    for (const auto& point : cloud.points)
        std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    return 0;
}
**/

编译test.cpp时需要手动链接相应库，先用 dpkg 找路径:
dpkg -L libpcl-dev | grep pcd_io.h    , 随后终端会输出对应的路径，进行手动链接。同时记得PCL还依赖于Eigen,lab1已经搭建了Eigen环境，编译test.cpp时一样要指定包含路径(用dpkg -L libeigen3-dev | grep Eigen/Core找路径)，最终我的OS上的编译命令如下(大概率你的也是):
g++ test.cpp -o test -I/usr/include/pcl-1.10 -lpcl_io -lpcl_common -I/usr/include/eigen3
    
我在lab2项目根目录下已经写好了test.cpp的构建运行脚本 run.sh ，理论上来说你只要照着lab1和lab2搭建好了Eigen和PCL库，直接使用 bash run.sh ，就可以运行出对应的结果
```



### 2. 按照如下流程完成lab2的第4点(实现点云图像拼接)

1. 进入 Lab2_JoinMap 目录下，先修改CMakeLists.txt文件中的C++标准，从11改到14

2. 标准步骤:  cmake .   ->  make  ->   ./joinMap   但我已经写好了shell脚本，如下:
   如果想要先删除之前的构建依赖，重新构建并运行程序，用 bash build_run.sh ,如果想不构建而直接运行程序，直接用 bash run.sh 即可。我建议给老师展示的时候，运行 bash_run.sh，这样过程完整一些
   
3. 上述步骤执行完后，终端会输出如下结果:
   
   ![image-20240607164438559](C:\Users\14811\AppData\Roaming\Typora\typora-user-images\image-20240607164438559.png)
   
4. ① apt install pcl_tools(第一次安装一次就行了)   

   ② pcl_viewer map.pcd    ，然后显示出结果如下:
   ![image-20240607164215391](C:\Users\14811\AppData\Roaming\Typora\typora-user-images\image-20240607164215391.png)



### 3. 实验课注意事项

实验指导书上的要求应该就上面这两张图片中的结果就行了，后面还有一个选择的要求，叫"双目识别系统"之类的，他问你做没做就说没做。如果他问你实验原理之类的，那就需要看看实验指导pdf以及我的实验报告的对应回答

