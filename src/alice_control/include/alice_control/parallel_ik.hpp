#pragma once

#include <Eigen/Dense>
#include <vector>

namespace parallel_ik {

    class ParallellIK {
    public:
        /**
         * @brief Construtor da classe de cinemática inversa com paralelogramo.
         */
        ParallellIK();

        /**
         * @brief Resolve a Cinemática Inversa incluindo as juntas do paralelogramo.
         * @param p_body Posição atual do tronco do robô.
         * @param R_body Orientação atual do tronco do robô.
         * @param p_foot_target Posição alvo desejada do pé.
         * @param R_foot_target Orientação alvo desejada do pé.
         * @param joint_angles Vetor que será preenchido com as 6 juntas calculadas:
         *                     [roll_tronco, pitch_tronco, -pitch_tronco, pitch_pe, -pitch_pe, roll_pe]
         * @return true se a solução foi encontrada com sucesso, false caso contrário.
         */
        bool solve(
            const Eigen::Vector3d & p_body,
            const Eigen::Quaterniond & R_body,
            const Eigen::Vector3d & p_foot_target,
            const Eigen::Quaterniond & R_foot_target,
            std::vector<double> & joint_angles);
    };

} // namespace parallel_ik
