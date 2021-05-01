// Because of Mosek complications, we don't use static library if Mosek is used.
#include <igl/writeMESH.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/readOFF.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>

using namespace Eigen;
using namespace std;

Eigen::MatrixXd V,W,U,C,M;
Eigen::MatrixXi T,F,BE;
Eigen::VectorXi P;

Eigen::MatrixXd TV;
Eigen::MatrixXi TT;
Eigen::MatrixXi TF;

int main(int argc, char * argv[]){

  igl::readOFF("../data/Dog_v2.off",V,F);
  
  igl::copyleft::tetgen::tetrahedralize(V,F,"", TV,TT,TF);
  igl::writeMESH("../data/cylinder.mesh", TV, TT, TF);

  return 0;

}