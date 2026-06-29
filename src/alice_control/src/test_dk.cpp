#include "alice_control/dk.hpp"
#include <iostream>
#include <vector>
#include <string>

int main() {
    std::cout << "--- TESTANDO HARDCODED ---" << std::endl;
    Eigen::Vector3d pos = dk::calcularPosicaoPe(30.0509, 5.38931, -7.08896, -5.38931, true);
    std::cout << "calcularPosicaoPe(0, 0, 0, 0):" << std::endl;
    std::cout << "X: " << pos.x() << std::endl;
    std::cout << "Y: " << pos.y() << std::endl;
    std::cout << "Z: " << pos.z() << std::endl;

    std::cout << "\n--- TESTANDO URDF ---" << std::endl;
    // Defina as juntas base e a junta final da perna aqui
    std::vector<std::pair<std::string, std::string>> joints = {
        {"base_link_l_up_leg_conn_joint", "l_up_leg_conn_l_up_back_leg_joint"},
        {"l_up_leg_conn_l_up_back_leg_joint", "l_up_back_leg_l_knee_joint"},
        {"l_up_back_leg_l_knee_joint", "l_knee_l_dn_back_leg_joint"},
        {"l_knee_l_dn_back_leg_joint", "l_dn_back_leg_l_ankle_joint"},
        {"l_dn_back_leg_l_ankle_joint", "l_ankle_l_foot_joint"}
    };
    
    Eigen::MatrixXd res = dk::medirDistanciasJuntas(joints);
    
    if(res.rows() > 0) {
        std::cout << "medirDistanciasJuntas(base_link -> l_ankle_l_foot_joint):" << std::endl;
        for(size_t i = 0; i < res.rows(); ++i) {
            std::cout << "X: " << res(i,0) << std::endl;
            std::cout << "Y: " << res(i,1) << std::endl;
            std::cout << "Z: " << res(i,2) << std::endl;
        }
    } else {
        std::cout << "Erro: Nao foi possivel calcular pelo URDF (Verifique se o nome das juntas esta correto)." << std::endl;
    }

    return 0;
}
