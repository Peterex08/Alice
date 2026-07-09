#include "alice_control/ik.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
    std::cout << "--- TESTANDO IK (URDF Dinamico) ---" << std::endl;

    ik::InvKinematics solver;
    std::vector<double> joint_angles;

    // Body position and orientation
    Eigen::Vector3d p_body(0.0, 0.0, 0.0);
    Eigen::Quaterniond R_body = Eigen::Quaterniond::Identity();

    // Teste 1: Perna Reta (x=0.0, y=0.0, z=-0.620)
    // Z esperado em metros = -0.050 (base) - 0.265 (coxa) - 0.040 (joelho) - 0.265 (canela) = -0.620 m
    Eigen::Vector3d p_foot_target1(0.0, 0.0, -0.620);
    Eigen::Quaterniond R_foot_target1 = Eigen::Quaterniond::Identity();

    std::cout << "\nTeste 1: Perna Reta (x=0.0, y=0.0, z=-0.620)" << std::endl;
    if (solver.solve(p_body, R_body, p_foot_target1, R_foot_target1, joint_angles)) {
        std::cout << "Sucesso!" << std::endl;
        std::cout << "q_roll_tronco:   " << joint_angles[0] << " rad (" << joint_angles[0] * 180.0 / M_PI << " graus)" << std::endl;
        std::cout << "q_pitch_tronco:  " << joint_angles[1] << " rad (" << joint_angles[1] * 180.0 / M_PI << " graus)" << std::endl;
        std::cout << "q_pitch_pe:      " << joint_angles[2] << " rad (" << joint_angles[2] * 180.0 / M_PI << " graus)" << std::endl;
        std::cout << "q_roll_pe:       " << joint_angles[3] << " rad (" << joint_angles[3] * 180.0 / M_PI << " graus)" << std::endl;
    } else {
        std::cout << "Falha ao resolver IK!" << std::endl;
    }

    // Teste 2: Passo (x=0.100, y=0.050, z=-0.580)
    Eigen::Vector3d p_foot_target2(0.100, 0.050, -0.580);
    Eigen::Quaterniond R_foot_target2 = Eigen::Quaterniond::Identity();

    std::cout << "\nTeste 2: Passo (x=0.100, y=0.050, z=-0.580)" << std::endl;
    if (solver.solve(p_body, R_body, p_foot_target2, R_foot_target2, joint_angles)) {
        std::cout << "Sucesso!" << std::endl;
        std::cout << "q_roll_tronco:   " << joint_angles[0] << " rad (" << joint_angles[0] * 180.0 / M_PI << " graus)" << std::endl;
        std::cout << "q_pitch_tronco:  " << joint_angles[1] << " rad (" << joint_angles[1] * 180.0 / M_PI << " graus)" << std::endl;
        std::cout << "q_pitch_pe:      " << joint_angles[2] << " rad (" << joint_angles[2] * 180.0 / M_PI << " graus)" << std::endl;
        std::cout << "q_roll_pe:       " << joint_angles[3] << " rad (" << joint_angles[3] * 180.0 / M_PI << " graus)" << std::endl;
    } else {
        std::cout << "Falha ao resolver IK!" << std::endl;
    }

    return 0;
}
