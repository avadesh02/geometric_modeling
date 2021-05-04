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

std::vector<std::vector<double>> motion;
std::vector<int> CE_ind = {633, 39664, 32904, 28716, 5931, 3569, 3632, 
                            32099, 31372, 27125, 7050, 3014, 2040};

std::vector<std::string> joint_names = {"FL_HAA", "FL_KFE", "FL_FOOT", "FR_HAA" ,"FR_KFE", "FR_FOOT",
                                        "HL_HAA", "HL_KFE", "HL_FOOT", "HR_HAA", "HR_KFE", "HR_FOOT"};
int k;
Eigen::VectorXd q;
std::string rpath = "../data/solo12.urdf";
std::string motion_file = "../motions/jump.txt";

std::vector<std::vector<double>> read_motion(std::string file_name){
    fstream newfile;
    newfile.open(file_name,ios::in); //open a file to perform read operation using file object

    std::vector<std::vector<double>> q_vec;

    if (newfile.is_open()){   //checking whether the file is open
        string tp;
        while(getline(newfile, tp)){ //read data from file object and put it into string.
            vector <double> OutputVertices;
            istringstream ss(tp);
            copy(
            istream_iterator <double> ( ss ),
            istream_iterator <double> (),
            back_inserter( OutputVertices )
            );
            q_vec.push_back(OutputVertices);
        }
        newfile.close(); //close the file object.
    };

    return q_vec;
}


bool pre_draw(igl::opengl::glfw::Viewer & viewer){

    for (unsigned i = 0; i < motion[k].size(); ++i){
        if (i < 3 || i > 6){
            q(i) = motion[k][i];
        }
        // hack because of difference in co ordinate frames 
        // of mesh and urdf
        q(0) = motion[k][1]; q(1) = -motion[k][0];
    }
    std::string rpath = "../data/solo12.urdf";
    fk::ForwardKinematics fk(rpath, joint_names, V, CE_ind, BE);
    MatrixXd CT(CE_ind.size(), V.cols());
    MatrixXd T_mat(BE.rows()*(V.cols()+1),V.cols());

    fk.compute(q, CT, T_mat);
    U = M*T_mat;
    viewer.data().set_vertices(U);
    // viewer.data().set_edges(CT,BE,sea_green);
    k = (k < motion.size() - 1 ? k + 1 : 0);
}

int main(int argc, char *argv[]){
    
    k = 0;
    motion = read_motion(motion_file);
    std::cout << "finished reading motion .." << std::endl;

    q.resize(19);
    q <<  0.2, 0.4, 0.24, 0.0, 0.0, -0.707107, 0.707107, 
          0.8, 0.8, -1.6, 0.0, 1.5, -1.6, 
          0.0, -0.8, 1.6, 0.0, -0.8, 1.6;

    BE.resize(12,2);
    BE << 0,1,
          1,2, 
          2,3,
          0,4,
          4,5,
          5,6,
          0,7,
          7,8,
          8,9,
          0,10,
          10,11,
          11,12;

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

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(U, F);
    // viewer.data().set_data(W.col(selected));
    // viewer.data().set_edges(C,BE,sea_green);
    viewer.callback_pre_draw = &pre_draw;
    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    viewer.data().line_width = 1;
    viewer.core().animation_max_fps = 60.;
    viewer.core().is_animating = true;
    viewer.launch();

    return 0;
}
