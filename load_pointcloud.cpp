#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
// #include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace Eigen;

static pcl::PointCloud<pcl::PointXYZI> global_map;

int main( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"Usage: error"<<endl;
        return 1;
    }
    ifstream fin( argv[1] );
    if ( !fin )
    {
        cout<<"file "<<argv[1]<<" does not exist."<<endl;
        return 1;
    }
    // ifstream fin_lidar( argv[2] );
    // if ( !fin_lidar )
    // {
    //     cout<<"file "<<argv[2]<<" does not exist."<<endl;
    //     return 1;
    // }

    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量
    while ( !fin.eof() /*&& !fin_lidar.eof()*/)
    {
        // std::cout << "frame number: " << vertexCnt << std::endl;
        vertexCnt++;
        string name;
        fin>>name;
        // std::cout << "vertex or edge: " << name << std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_transform_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        if ( name == "VERTEX_SE3:QUAT" )
        {
            int index = 0;
            fin>>index;
            std::cout << "frame ID: " << index << std::endl;
            if(index == 0 /*|| index > 990*/){
                continue;
            }
            // if(index < 2500){
            //     continue;
            // }
            // if(index > 400){
            //     break;
            // }
            double data[7];
            for ( int i=0; i<7; i++ ){
                fin>>data[i];
            }

            Eigen::Matrix4f transform_matric = Eigen::Matrix4f::Identity();
            Quaterniond t_Q(data[6], data[3], data[4], data[5]);; // (1, 0.031702, 0.425594, 0.125068);
            Eigen::Matrix3d tmp_matric = t_Q.matrix();
            transform_matric(0, 3) = data[0];
            transform_matric(1, 3) = data[1];
            transform_matric(2, 3) = data[2];
            for(int i=0; i<3; ++i){
                for(int j=0; j<3; ++j){
                    transform_matric(i, j) = tmp_matric(i, j);
                }
            }
            // std::cout << transform_matric << std::endl;

            // pcl::PointXYZI tmp_point;
            // pcl::PointCloud<pcl::PointXYZI> scan_tmp;
            // double point_data[4];
            // for(int i=0; i<250000; ++i){
            //     for ( int j=0; j<4; j++ ){
            //         fin_lidar >> point_data[j];
            //     }
            //     tmp_point.x = point_data[0];
            //     tmp_point.y = point_data[1];
            //     tmp_point.z = point_data[2];
            //     tmp_point.intensity = point_data[3];
            //     scan_tmp.push_back(tmp_point);
            // }
            pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
            string pointPath = "/home/lgw/Documents/project/ndt_mapping/no_gps/ndt_1/frame_1/" + std::to_string(index - 1) + ".pcd";
            pcl::io::loadPCDFile<pcl::PointXYZI>(pointPath, *scan_ptr);
            // pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_transform_ptr(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*scan_ptr, *tmp_transform_ptr, transform_matric);
            
            // if ( index == 935 || index == 936){
            //     v->setFixed(true);
            // }
        }
        global_map += *tmp_transform_ptr;
        // if(global_map.size() > 250000 * 10 && global_map.size() < 250000 * 20){
        //     pcl::io::savePCDFileBinary("/home/lgw/Documents/slambook-master/ch11/build/test.pcd", global_map);
        // }

        if ( !fin.good() ) break;
    }
    pcl::io::savePCDFileBinary("/home/lgw/Documents/slambook-master/ch11/build/test.pcd", global_map);

    return 0;
}
