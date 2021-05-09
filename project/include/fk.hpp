// This file contains the forward kinematics for a given robot urdf

#ifndef _FORWARD_KINEMATICS_
#define _FORWARD_KINEMATICS_

#include <iostream>

// for forward kinematics of robot
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"

namespace fk{

    class ForwardKinematics{

        public:
            ForwardKinematics(std::string urdf_path, std::vector<std::string> joint_names,
                              Eigen::MatrixXd V, std::vector<int> CE_ind, Eigen::MatrixXi BE);

            // computes the relative positions of handles wrt to initial in SE3
            void compute(Eigen::VectorXd &q, Eigen::MatrixXd &CT, Eigen::MatrixXd &T_mat);

            void get_C(Eigen::MatrixXd &C);

            Eigen::Quaterniond euler2Quaternion( const double roll,
                                                const double pitch,
                                                const double yaw );

        private:
            
            // robot model
            pinocchio::Model rmodel_;
            // robot data
            pinocchio::Data rdata_;
            // joint names
            std::vector<std::string> joint_names_;
            // Constraint vertices (handles)
            Eigen::MatrixXd C_;
            // current location of constraint handles
            Eigen::MatrixXd CT_;
            Eigen::MatrixXd V_;
            Eigen::MatrixXi BE_;
            // The intial position of the handles
            std::vector<Eigen::Affine3d> T0_;
            // the current position of handles based on joint position
            // relative to the initial postion
            std::vector<Eigen::Affine3d> T_;
            Eigen::Affine3d pitch_corr;


    };
}

#endif