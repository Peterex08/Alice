#include "alice_control/ik.hpp"
#include "alice_control/dk.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ik {

    InvKinematics::InvKinematics() {}

    bool InvKinematics::solve(
        const Eigen::Vector3d & p_body,
        const Eigen::Quaterniond & R_body,
        const Eigen::Vector3d & p_foot_target,
        const Eigen::Quaterniond & R_foot_target,
        std::vector<double> & joint_angles)
    {
        // 1. PARÂMETROS FIXOS DA GEOMETRIA (Extraídos da Cinemática Direta via URDF)
        Eigen::MatrixXd diff = dk::medirDistanciasJuntas({
            {"base_link_l_up_leg_conn_joint", "l_up_leg_conn_l_up_back_leg_joint"},
            {"l_up_leg_conn_l_up_back_leg_joint", "l_up_back_leg_l_knee_joint"},
            {"l_up_back_leg_l_knee_joint", "l_knee_l_dn_back_leg_joint"},
            {"l_knee_l_dn_back_leg_joint", "l_dn_back_leg_l_ankle_joint"},
            {"l_dn_back_leg_l_ankle_joint", "l_ankle_l_foot_joint"}
        });

        if (diff.rows() < 5) {
            throw std::runtime_error("Erro ao obter distancias do URDF.");
        }

        Eigen::MatrixXd diff_mm = diff * 1000.0;

        // 1. Converte o target global para o referencial local do Tronco
        Eigen::Matrix3d R_b = R_body.toRotationMatrix();
        Eigen::Vector3d p_rel = R_b.transpose() * (p_foot_target - p_body);
        Eigen::Matrix3d R_rel = R_b.transpose() * R_foot_target.toRotationMatrix();

        double X = p_rel.x() * 1000.0;
        double Y = p_rel.y() * 1000.0;
        double Z = p_rel.z() * 1000.0;

        try {
            // 2. Extrai o Roll do alvo (já que o paralelogramo trava o Pitch e Yaw)
            double roll_target = atan2(R_rel(2, 1), R_rel(2, 2));

            // 3. Resolve Joint 1: Roll do Tronco
            // O desvio em Y é gerado exclusivamente pelo Roll inicial. 
            // O offset Z do hip_roll → hip_pitch é diff_mm(0,2) ≈ -50mm
            double hip_offset_z = diff_mm(0,2);  // -50 mm
            double q_roll_tronco = atan2(Y, -(Z - hip_offset_z));

            // 4. Calcula o vetor Z "virtual" que desceu na perna, removendo o efeito do Roll
            double Vz = -sqrt(Y * Y + (Z - hip_offset_z) * (Z - hip_offset_z));

            // 5. Mapeia para um problema planar 2D (Interseção de círculos)
            // O ponto de partida do 2-link é após o hip_pitch joint.
            // O ponto de chegada é o ankle joint (antes do foot offset).
            // Offset constante Z que NÃO faz parte dos dois elos rotativos:
            //   knee_offset (diff_mm(2,2) ≈ -40mm) + foot_offset (diff_mm(4,2) ≈ -50mm)
            double knee_offset_z = diff_mm(2,2);   // -40 mm
            double foot_offset_z = diff_mm(4,2);    // -50 mm
            double offset_z = knee_offset_z + foot_offset_z;  // -90 mm (constantes que deslocam verticalmente)

            double X_prime = -X;
            double Y_prime = -Vz + offset_z;  // remove os offsets constantes do comprimento virtual

            // Tamanhos dos elos principais: coxa (265mm) e canela (265mm)
            double L1 = -diff_mm(1,2);  // coxa: 265 mm
            double L2 = -diff_mm(3,2);  // canela: 265 mm

            double D = sqrt(X_prime * X_prime + Y_prime * Y_prime);

            // Proteção de singularidade e alcance máximo/mínimo
            if (D > L1 + L2) { D = L1 + L2 - 1e-6; }
            if (D < std::abs(L1 - L2)) { D = std::abs(L1 - L2) + 1e-6; }

            // 6. Resolve Joint 2: Pitch do Tronco
            double alpha = atan2(X_prime, Y_prime);
            double cos_beta = (D * D + L1 * L1 - L2 * L2) / (2.0 * D * L1);
            cos_beta = std::max(-1.0, std::min(1.0, cos_beta));
            double beta = acos(cos_beta);

            // alpha - beta faz o joelho dobrar para frente (direção correta).
            double q_pitch_tronco = alpha - beta; 

            // 7. Resolve Joint 3: Pitch do Pé
            double X3_prime = X_prime - L1 * sin(q_pitch_tronco);
            double Y3_prime = Y_prime - L1 * cos(q_pitch_tronco);
            double q_pitch_pe = atan2(X3_prime, Y3_prime);

            // 8. Resolve Joint 4: Roll do Pé
            // Como a orientação de roll se soma (q_roll_tronco + q_roll_pe = roll_target)
            double q_roll_pe = roll_target - q_roll_tronco;

            // Limpa e popula o vetor de juntas (Retornando os 4 DOFs independentes)
            // Caso precise replicar os motores do paralelogramo (ex: q2 e -q2), faça a atribuição após o solver.
            joint_angles = {q_roll_tronco, q_pitch_tronco, q_pitch_pe, q_roll_pe};

            return true;

        } catch (const std::exception & e) {
            std::cerr << "Erro na IK: " << e.what() << '\n';
            return false;
        }
    }

} // namespace ik
