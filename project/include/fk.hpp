// This file contains the forward kinematics for a given robot urdf

#ifndef _FORWARD_KINEMATICS_
#define _FORWARD_KINEMATICS_

#include <iostream>

// for forward kinematics of robot
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/data.hpp"


namespace fk{

    class ForwardKinematics{

        public:
            ForwardKinematics(std::string urdf_path);

        
        private:
            
            // robot model
            pinocchio::Model rmodel_;
            // robot data
            pinocchio::Data rdata_;
    };
}

#endif