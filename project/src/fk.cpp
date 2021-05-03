#include <fk.hpp>


namespace fk{

    ForwardKinematics::ForwardKinematics(std::string urdf_path, std::vector<std::string> joint_names,
                              Eigen::MatrixXd V, std::vector<int> CE_ind, Eigen::MatrixXi BE):
        joint_names_(joint_names),
        V_(V),
        BE_(BE)
    {
        pinocchio::urdf::buildModel(urdf_path,pinocchio::JointModelFreeFlyer(), rmodel_) ;
        pinocchio::Data rdata_tmp(rmodel_);
        rdata_ = rdata_tmp;

        C_.resize(CE_ind.size(), V.cols());
        T0_.resize(CE_ind.size());
        T_.resize(CE_ind.size());
        for (unsigned i = 0; i < CE_ind.size(); ++i){
            C_.row(i) = V.row(CE_ind[i]);
            T0_[i] = Eigen::Affine3d::Identity();
            T0_[i].translate(Eigen::Vector3d(C_.row(i)));
            T_[i] = Eigen::Affine3d::Identity();
        }
        CT_ = C_;
    };

    void ForwardKinematics::get_C(Eigen::MatrixXd &C){
        C = C_;
    };

    void ForwardKinematics::compute(Eigen::VectorXd &q, Eigen::MatrixXd &CT, Eigen::MatrixXd &T_mat){

        pinocchio::forwardKinematics(rmodel_, rdata_, q);
        pinocchio::updateFramePlacements(rmodel_, rdata_);
        
        CT.row(0) = C_.row(0);
        for (unsigned i = 0; i < joint_names_.size(); ++i){
            
            T_[i+1].translation() = rdata_.oMf[rmodel_.getFrameId(joint_names_[i])].translation();
            // T_[i].rotate(rdata_.oMf[rmodel_.getFrameId(joint_names_[i])].rotation());
            CT.row(i+1) = Eigen::Vector3d(T_[i+1].translation());
            // computing the relative translation vector
            T_[i+1] = T_[i+1]*T0_[i+1].inverse();

        }

        int dim = V_.cols();

        for(int e = 0;e<BE_.rows();e++)
            {
                T_mat.block(e*(dim+1),0,dim+1,dim) =
                    T_[e+1].matrix().transpose().block(0,0,dim+1,dim);
            }
    };
    

}