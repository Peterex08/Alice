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

    IkResult calc_ik_analitica_perna(double x_alvo, double y_alvo, double z_alvo, bool use_degrees) {
        // 1. PARÂMETROS FIXOS DA GEOMETRIA (Extraídos da Cinemática Direta via URDF)
        Eigen::MatrixXd diff = dk::medirDistanciasJuntas({
            {"base_link_l_up_leg_conn_joint", "l_up_leg_conn_l_up_back_leg_joint"},
            {"l_up_leg_conn_l_up_back_leg_joint", "l_up_back_leg_l_knee_joint"},
            {"l_up_back_leg_l_knee_joint", "l_knee_l_dn_back_leg_joint"},
            {"l_knee_l_dn_back_leg_joint", "l_dn_back_leg_l_ankle_joint"}
        });

        if (diff.rows() < 4) {
            throw std::runtime_error("Erro ao obter distancias do URDF.");
        }

        // Baseando na logica Python original e adaptando pro output de medirDistanciasJuntas
        double HBb_z = diff(0, 2); 
        double L_coxa = std::abs(diff(1, 2)); 
        double offset_joelho_z = diff(2, 2); 
        double L_canela = std::abs(diff(3, 2)); 

        // 2. ISOLAR O VETOR DA PERNA (Removendo o offset do quadril)
        double X_v = x_alvo;
        double Y_v = y_alvo;
        double Z_v = z_alvo - HBb_z;

        // 3. SOLVER DO ROLL DO QUADRIL (phi_joelho)
        // Qual o angulo lateral necessario para alinhar a perna com o alvo no eixo Y?
        double phi_j_rad = std::atan2(Y_v, -Z_v);

        // 4. DESENROLAR PARA O PLANO 2D (Sagital)
        // Ao "desfazer" o roll, o Y se torna 0 e a perna vira um mecanismo 2D
        double X_2d = X_v;
        double Z_2d = -std::sqrt(Y_v * Y_v + Z_v * Z_v); 

        // Compensar o pequeno degrau de 40mm do joelho no plano Z
        double Z_2d_eff = Z_2d - offset_joelho_z;

        // 5. SOLVER DO PITCH (Plano 2D - Lei dos Cossenos)
        // Distancia reta do quadril ate o tornozelo no plano 2D
        double D_quadrado = X_2d * X_2d + Z_2d_eff * Z_2d_eff;

        // Verificacao de seguranca (Singularidade / Fora de alcance)
        double L_max = (L_coxa + L_canela) * (L_coxa + L_canela);
        double L_min = (L_coxa - L_canela) * (L_coxa - L_canela);
        if (D_quadrado > L_max || D_quadrado < L_min) {
            std::cerr << "AVISO: O ponto alvo esta fora do alcance fisico da perna." << std::endl;
        }

        // C e o cosseno do angulo interno formado pela dobra do joelho
        double C = (D_quadrado - L_coxa * L_coxa - L_canela * L_canela) / (2 * L_coxa * L_canela);
        // Prevencao contra erros de precisao do float
        if (C > 1.0) C = 1.0;
        if (C < -1.0) C = -1.0;

        // alpha e a diferenca relativa entre o angulo da coxa e da canela
        double alpha = std::acos(C);

        // Variaveis auxiliares para o sistema linear geometrico
        double A = L_coxa + L_canela * std::cos(alpha);
        double B = L_canela * std::sin(alpha);

        // Resolver o Pitch da Coxa (theta_joelho)
        double sin_theta_j = (A * X_2d - B * Z_2d_eff) / D_quadrado;
        double cos_theta_j = (-B * X_2d - A * Z_2d_eff) / D_quadrado;
        double theta_j_rad = std::atan2(sin_theta_j, cos_theta_j);

        // Resolver o Pitch da Canela (theta_pe)
        // Na sua arquitetura, theta_p e absoluto, entao basta subtrair alpha do quadril
        double theta_p_rad = theta_j_rad - alpha;

        // 6. MANTER O PE RETO
        // Para que a sola do robo fique perfeitamente paralela ao chao lateralmente,
        // o Roll do tornozelo deve ser exatamente o inverso do Roll do quadril.
        double phi_p_rad = -phi_j_rad;

        IkResult result;
        if (use_degrees) {
            result.theta_joelho = theta_j_rad * 180.0 / M_PI;
            result.phi_joelho = phi_j_rad * 180.0 / M_PI;
            result.theta_pe = theta_p_rad * 180.0 / M_PI;
            result.phi_pe = phi_p_rad * 180.0 / M_PI;
        } else {
            result.theta_joelho = theta_j_rad;
            result.phi_joelho = phi_j_rad;
            result.theta_pe = theta_p_rad;
            result.phi_pe = phi_p_rad;
        }

        return result;
    }

} // namespace ik
