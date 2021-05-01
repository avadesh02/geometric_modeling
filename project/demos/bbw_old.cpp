#include <igl/boundary_conditions.h>
#include <igl/readMESH.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/bbw.h>
#include <igl/normalize_row_sums.h>
#include <igl/forward_kinematics.h>
#include <igl/directed_edge_parents.h>

#include <igl/lbs_matrix.h>
#include <igl/deform_skeleton.h>


#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>

using namespace Eigen;
using namespace std;

const Eigen::RowVector3d sea_green(70./255.,252./255.,167./255.);

Eigen::MatrixXd V,W,U,C,M;
Eigen::MatrixXi T,F,BE;
Eigen::VectorXi P;

int selected = 0;


Eigen::MatrixXd TV;
Eigen::MatrixXi TT;
Eigen::MatrixXi TF;

typedef std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> RotationList;

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


int main(int argc, char *argv[]){

    igl::readOFF("../data/cylinder.off",V,F);
    std::cout << "finished reading ..." << std::endl;

    U = V;

    C.resize(3,3);

    C.row(0) = V.row(4800);
    C.row(1) = V.row(2372);
    C.row(2) = V.row(4801);

    BE.resize(2,2);
    
    BE << 0, 1,
          1, 2;

    // List of boundary indices (aka fixed value indices into VV)
    VectorXi b;
    // List of boundary conditions of each weight function
    MatrixXd bc;
    igl::boundary_conditions(V,F,C,VectorXi(),BE,MatrixXi(),b,bc);

    std::cout << "vec" << VectorXi() << std::endl;

    igl::BBWData bbw_data;
    // only a few iterations for sake of demo
    bbw_data.active_set_params.max_iter = 8;
    bbw_data.verbosity = 2;
    if(!igl::bbw(V,F,b,bc,bbw_data,W))
    {
        return EXIT_FAILURE;
    }
    // Normalize weights to sum to one
    igl::normalize_row_sums(W,W);

    // precompute linear blend skinning matrix
    igl::lbs_matrix(V,W,M);

    //////////////////////////////////////////////////////////
    // bending the cylinder

    Vector3d dist1 = (C.row(1) - C.row(0));
    Vector3d dist2 = (C.row(2) - C.row(1));
    Affine3d T1 = Affine3d::Identity();
    Affine3d T2 = Affine3d::Identity();
    Affine3d T3 = Affine3d::Identity();

    T1.translate(dist1); 
    T3.translate(dist2);
    T2.rotate(euler2Quaternion(0.5*M_PI, 0.0, 0.0));

    std::cout << euler2Quaternion(0.0, 0.0, 0.5*M_PI) << std::endl;

    Affine3d T0 = Affine3d::Identity();
    T0.translate(Vector3d(C.row(2)));
    Affine3d T_ff = T1*T2*T3;

    const int dim = C.cols();
    MatrixXd T(BE.rows()*(dim+1),dim);
    for(int e = 0;e<BE.rows();e++)
    {
        Affine3d a = Affine3d::Identity();
        if (e == 1){
            a = T_ff*T0.inverse();
        }
        // a.translate(vT[e]);
        // a.rotate(vQ[e]);
        T.block(e*(dim+1),0,dim+1,dim) =
            a.matrix().transpose().block(0,0,dim+1,dim);

    }
    // Compute deformation via LBS as matrix multiplication
    U = M*T;
    MatrixXd CT;
    MatrixXi BET;
    igl::deform_skeleton(C,BE,T,CT,BET);

    // std::cout << C << std::endl;

    // std::cout << CT << std::endl;

    // std::cout << BET << std::endl;

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(U, F);
    viewer.data().set_data(W.col(selected));
    viewer.data().set_edges(CT,BET,sea_green);
    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    viewer.data().line_width = 1;
    viewer.launch();

    return 0;
}


// std::cout << "FK " << Quaterniond((T_ff*T0.inverse()).rotation()).vec() <<  std::endl;

    // RotationList anim_pose(2);
    // anim_pose[0] = euler2Quaternion(0, 0.0, 0.0);
    // anim_pose[1] = euler2Quaternion(0.5*M_PI, 0.0, 0.0);

    // // retrieve parents for forward kinematics
    // igl::directed_edge_parents(BE,P);
    // RotationList vQ;
    // vector<Vector3d> vT;
    // igl::forward_kinematics(C,BE,P,anim_pose,vQ,vT);

    // std::cout << "igl FK" << vQ[1].vec() << " " << std::endl;