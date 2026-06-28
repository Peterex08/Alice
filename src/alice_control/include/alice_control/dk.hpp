#pragma once
#include <Eigen/Dense>

namespace dk {

    /**
     * @brief Calcula a posição final (X, Y, Z) do pé usando Cinemática Direta.
     * * @param roll_tronco_deg Ângulo de roll do tronco em graus.
     * @param pitch_tronco_deg Ângulo de pitch do tronco em graus.
     * @param roll_pe_deg Ângulo de roll do pé em graus.
     * @param pitch_pe_deg Ângulo de pitch do pé em graus.
     * @return Eigen::Vector3d Contendo as coordenadas finais (X, Y, Z).
     */
    Eigen::Vector3d calcularPosicaoPe(double roll_tronco_deg, double pitch_tronco_deg, double roll_pe_deg, double pitch_pe_deg);

} // namespace dk