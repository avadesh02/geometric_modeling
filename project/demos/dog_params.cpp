#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

// contains indices of the mesh vertex that are handle points 
Eigen::MatrixXi BE;


int main(int argc, char **argv){

        std::vector<int> CE_ind = {0,28821, 33603};


        BE.resize(2,2);
        BE << 0,1, 
            1,2;

}