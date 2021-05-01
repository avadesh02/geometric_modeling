// This is a demo to test the fk

#include <iostream>
#include <fk.hpp>
#include <igl/readOFF.h>

// #include "dog_params.cpp"

std::vector<int> CE_ind = {0,28821, 33603};
std::vector<std::string> joint_names = {"FL_FOOT", "FL_KFE", "FL_HAA"};

Eigen::MatrixXi F, BE;
Eigen::MatrixXd V,C, CT, T_mat;

Eigen::VectorXd q;

Eigen::Quaterniond
euler2Quaternion( const double roll,
                  const double pitch,
                  const double yaw )
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

int main(int argc, char **argv){

    BE.resize(2,2);
    BE << 0,1, 
          1,2;

    // reading mesh
    igl::readOFF("../data/Dog_v2.off",V,F);

    std::string rpath = "../data/solo12.urdf";

    fk::ForwardKinematics fk(rpath, joint_names, V, CE_ind, BE);

    q.resize(19);
    q <<  0.2, 0.0, 0.24, 0.0, 0.0, 0.0, 1.0, 
          0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 
          0.0, -0.8, 1.6, 0.0, -0.8, 1.6;

    T_mat.resize(BE.rows()*(V.cols()+1),V.cols());
    CT.resize(CE_ind.size(), V.cols());
    fk.compute(q, CT, T_mat);

    std::cout << euler2Quaternion(0.0, 0.5*M_PI, 0.0).vec() << " " << euler2Quaternion(0.0, 0.5*M_PI, 0.0).w()<< std::endl;


    return 0;

}