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