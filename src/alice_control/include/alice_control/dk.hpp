#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <utility>

namespace dk {

    /**
     * @brief Calcula a posição final (X, Y, Z) do pé usando Cinemática Direta.
     * @param roll_tronco Ângulo de roll do tronco.
     * @param pitch_tronco Ângulo de pitch do tronco.
     * @param roll_pe Ângulo de roll do pé.
     * @param pitch_pe Ângulo de pitch do pé.
     * @param use_degrees Define se as entradas estão em graus (true) ou radianos (false). Padrão é false.
     * @return Eigen::Vector3d Contendo as coordenadas finais (X, Y, Z).
     */
    Eigen::Vector3d calcularPosicaoPe(double roll_tronco, double pitch_tronco, double roll_pe, double pitch_pe, bool use_degrees = false);

    /**
     * @brief Lê o URDF e mede as distâncias relativas X, Y, Z entre pares de juntas.
     * Mantém as medidas em cache para evitar releituras do URDF e acelerar consultas repetidas.
     * @param joint_pairs Vetor de pares de strings (junta_inicial, junta_final).
     * @return Eigen::MatrixXd Matriz X por 3 contendo as distâncias (X, Y, Z) para cada par.
     */
    Eigen::MatrixXd medirDistanciasJuntas(const std::vector<std::pair<std::string, std::string>>& joint_pairs);

} // namespace dk