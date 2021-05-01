#include <fk.hpp>


namespace fk{

    ForwardKinematics::ForwardKinematics(std::string urdf_path)
    {
        pinocchio::urdf::buildModel(urdf_path,pinocchio::JointModelFreeFlyer(), rmodel_) ;
        pinocchio::Data rdata_tmp(rmodel_);
        rdata_ = rdata_tmp;

    };

}