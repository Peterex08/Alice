#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace dk {
    // Função que recebe os ângulos em graus e retorna a posição final do pé (X, Y, Z)
    Eigen::Vector3d calcularPosicaoPe(double roll_tronco_deg, double pitch_tronco_deg, double roll_pe_deg, double pitch_pe_deg) {
        // Conversão para radianos
        double roll_tronco = roll_tronco_deg * M_PI / 180.0;
        double pitch_tronco = pitch_tronco_deg * M_PI / 180.0;
        double roll_pe = roll_pe_deg * M_PI / 180.0;
        double pitch_pe = pitch_pe_deg * M_PI / 180.0;

        // Definição das matrizes de transformação homogênea (4x4)
        Eigen::Matrix4d Rtronco;
        Rtronco << 1, 0, 0, 0,
                    0, std::cos(roll_tronco), -std::sin(roll_tronco), 0,
                    0, std::sin(roll_tronco), std::cos(roll_tronco), -50.0,
                    0, 0, 0, 1;

        Eigen::Matrix4d Ptronco;
        Ptronco << std::cos(pitch_tronco), 0, std::sin(pitch_tronco), 0,
                    0, 1, 0, 0,
                    -std::sin(pitch_tronco), 0, std::cos(pitch_tronco), -265.0,
                    0, 0, 0, 1;

        Eigen::Matrix4d Pjoelhoup;
        Pjoelhoup << std::cos(-pitch_tronco), 0, std::sin(-pitch_tronco), 0,
                        0, 1, 0, 0,
                        -std::sin(-pitch_tronco), 0, std::cos(-pitch_tronco), -40.0,
                        0, 0, 0, 1;

        Eigen::Matrix4d Ppe;
        Ppe << std::cos(pitch_pe), 0, std::sin(pitch_pe), 0,
                0, 1, 0, 0,
                -std::sin(pitch_pe), 0, std::cos(pitch_pe), -265.0,
                0, 0, 0, 1;

        Eigen::Matrix4d Ppeup;
        Ppeup << std::cos(-pitch_pe), 0, std::sin(-pitch_pe), 0,
                    0, 1, 0, 0,
                    -std::sin(-pitch_pe), 0, std::cos(-pitch_pe), -50.0,
                    0, 0, 0, 1;

        Eigen::Matrix4d Rpe;
        Rpe << 1, 0, 0, 0,
                0, std::cos(roll_pe), -std::sin(roll_pe), 0,
                0, std::sin(roll_pe), std::cos(roll_pe), -60.7,
                0, 0, 0, 1;

        // Multiplicação matricial acumulada (Cinemática Direta)
        // T_cum = Rtronco * Ptronco * Pjoelhoup * Ppe * Ppeup * Rpe
        Eigen::Matrix4d T_cum = Rtronco * Ptronco * Pjoelhoup * Ppe * Ppeup * Rpe;

        // Ponto de origem
        Eigen::Vector4d ponto_inicial(0.0, 0.0, 0.0, 1.0);

        // Calcula a posição final global
        Eigen::Vector4d ponto_final = T_cum * ponto_inicial;

        // Retorna apenas a translação (X, Y, Z) cortando a 4ª dimensão
        return ponto_final.head<3>();
    }
} // namespace dk