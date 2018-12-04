#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

std::string novatel_pose = "/home/lgw/Documents/project/ndt_mapping/no_gps/ndt_1/ndt_pose.txt"; // .binary";
std::ofstream novatel_pose_outFile(novatel_pose.c_str(), std::ios::out);

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

    while ( !fin.eof() )
    {        
        int name;
        fin>>name;

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

        {
            std::stringstream ss;    
            ss << std::setprecision(12) << std::fixed;
            for(int i=0; i<3; ++i){
                for(int j=0; j<4; ++j){
                    ss << transform_matric(i, j) << " ";
                }
            }
            novatel_pose_outFile << ss.str() << std::endl;
        }

        if ( !fin.good() ) break;
    }

    return 0;
}
