#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <string>

#include "alice_control/parallel_ik.hpp"
#include "alice_control/ik.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace parallel_ik{
    
    ParallellIK::ParallellIK() {}

    bool ParallellIK::solve(
        const Eigen::Vector3d & p_body,
        const Eigen::Quaterniond & R_body,
        const Eigen::Vector3d & p_foot_target,
        const Eigen::Quaterniond & R_foot_target,
        std::vector<double> & joint_angles)
    {

        // 1. Resolver a IK principal (4 juntas)
        ik::InvKinematics ik_solver;
        std::vector<double> main_joints;

        if (!ik_solver.solve(p_body, R_body, p_foot_target, R_foot_target, main_joints)) {
            return false;
        }

        joint_angles = {main_joints[0], main_joints[1], -main_joints[1], main_joints[2], -main_joints[2], main_joints[3]};

        // As juntas principais (q0 a q3) já estão no vetor main_joints.
        // Agora precisamos calcular as juntas do paralelogramo (q4 a q7).

        // ... (Falta calcular as juntas do paralelogramo)
        
        return true;
    }

    
} //namespace parallel_ik