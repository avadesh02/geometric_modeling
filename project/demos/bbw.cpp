#include <igl/boundary_conditions.h>
#include <igl/readMESH.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/bbw.h>
#include <igl/normalize_row_sums.h>
#include <igl/forward_kinematics.h>
#include <igl/directed_edge_parents.h>

#include <igl/lbs_matrix.h>
#include <igl/deform_skeleton.h>

#include <fk.hpp>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>

using namespace Eigen;
using namespace std;

const Eigen::RowVector3d sea_green(70./255.,252./255.,167./255.);

Eigen::MatrixXd V,W,U,C,M, CT, T_mat;
Eigen::MatrixXi T,F,BE;
Eigen::VectorXi P;

int selected = 0;

std::vector<int> CE_ind = {30770, 28646, 30844, 4001, 3581, 5761, 27125, 31390,  32100, 2038, 2749, 7052};

std::vector<std::string> joint_names = {"FL_FOOT", "FL_KFE", "FL_HAA", "FR_FOOT", "FR_KFE", "FR_HAA",
                                        "HL_FOOT", "HL_KFE", "HL_HAA", 
                                            "HR_FOOT", "HR_KFE", "HR_HAA"};
Eigen::VectorXd q;
std::string rpath = "../data/solo12.urdf";
int main(int argc, char *argv[]){

    BE.resize(8,2);
    BE << 0,1, 
          1,2, 
          3,4, 
          4,5,
          6,7,
          7,8,
          9,10,
          10,11;

    igl::readOFF("../data/Dog_v1.off",V,F);
    U = V;
    std::cout << "finished reading ..." << std::endl;
    fk::ForwardKinematics fk(rpath, joint_names, V, CE_ind, BE);
    fk.get_C(C);

    // BBW
    VectorXi b;
    MatrixXd bc;
    igl::boundary_conditions(V,F,C,VectorXi(),BE,MatrixXi(),b,bc);
    igl::BBWData bbw_data;
    bbw_data.active_set_params.max_iter = 8;
    bbw_data.verbosity = 2;
    if(!igl::bbw(V,F,b,bc,bbw_data,W))
    {
        return EXIT_FAILURE;
    }

    igl::normalize_row_sums(W,W);
    igl::lbs_matrix(V,W,M);


    q.resize(19);
    q <<  0.2, 0.4, 0.24, 0.0, 0.0, -0.707107, 0.707107, 
          0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 
          0.0, -0.8, 1.6, 0.0, -0.8, 1.6;
        

    T_mat.resize(BE.rows()*(V.cols()+1),V.cols());
    CT.resize(CE_ind.size(), V.cols());

    fk.compute(q, CT, T_mat);
    
    U = M*T_mat;

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(U, F);
    // viewer.data().set_data(W.col(selected));
    viewer.data().set_edges(CT,BE,sea_green);
    // viewer.data().set_edges(C,BE,sea_green);

    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    viewer.data().line_width = 1;
    viewer.launch();

    return 0;
}
