#include "alice_control/ik.hpp"
#include <iostream>

int main() {
    std::cout << "--- TESTANDO IK (URDF Dinamico) ---" << std::endl;

    // IMPORTANTE: Como o URDF usa metros, os alvos devem ser passados em metros!
    // Teste 1: Perna perfeitamente esticada para baixo
    // Z esperado = -0.050 (base) - 0.265 (coxa) - 0.040 (joelho) - 0.265 (canela) = -0.620 m
    std::cout << "\nTeste 1: Perna Reta (x=0.0, y=0.0, z=-0.620)" << std::endl;
    ik::IkResult res1 = ik::calc_ik_analitica_perna(0.0, 0.0, -0.620, true);
    
    std::cout << "Theta Joelho (Pitch): " << res1.theta_joelho << " graus" << std::endl;
    std::cout << "Phi Joelho (Roll):   " << res1.phi_joelho << " graus" << std::endl;
    std::cout << "Theta Pe (Pitch):     " << res1.theta_pe << " graus" << std::endl;
    std::cout << "Phi Pe (Roll):       " << res1.phi_pe << " graus" << std::endl;

    // Teste 2: Passo para frente e levemente para a esquerda
    std::cout << "\nTeste 2: Passo (x=0.100, y=0.050, z=-0.580)" << std::endl;
    ik::IkResult res2 = ik::calc_ik_analitica_perna(0.100, 0.050, -0.580, true);
    
    std::cout << "Theta Joelho (Pitch): " << res2.theta_joelho << " graus" << std::endl;
    std::cout << "Phi Joelho (Roll):   " << res2.phi_joelho << " graus" << std::endl;
    std::cout << "Theta Pe (Pitch):     " << res2.theta_pe << " graus" << std::endl;
    std::cout << "Phi Pe (Roll):       " << res2.phi_pe << " graus" << std::endl;

    return 0;
}
