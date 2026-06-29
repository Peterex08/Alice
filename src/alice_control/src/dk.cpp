#include "alice_control/dk.hpp"
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <urdf/model.h>
#include <map>
#include <memory>
#include <cstdio>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace dk {
    // Helper para buscar transformacao absoluta da base ate a junta
    Eigen::Matrix4d getAbsoluteTransform(const std::shared_ptr<urdf::Model>& model, const std::string& joint_name) {
        urdf::JointConstSharedPtr joint = model->getJoint(joint_name);
        if (!joint) return Eigen::Matrix4d::Identity();

        urdf::LinkConstSharedPtr current_link = model->getLink(joint->child_link_name);
        Eigen::Matrix4d T_abs = Eigen::Matrix4d::Identity();

        while (current_link && current_link->parent_joint) {
            urdf::JointConstSharedPtr parent_joint = current_link->parent_joint;
            
            urdf::Pose pose = parent_joint->parent_to_joint_origin_transform;
            
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
            T.block<3,3>(0,0) = q.toRotationMatrix();
            T(0,3) = pose.position.x;
            T(1,3) = pose.position.y;
            T(2,3) = pose.position.z;

            T_abs = T * T_abs;
            
            current_link = model->getLink(parent_joint->parent_link_name);
        }
        return T_abs;
    }

    Eigen::MatrixXd medirDistanciasJuntas(const std::vector<std::pair<std::string, std::string>>& joint_pairs) {
        static std::shared_ptr<urdf::Model> model = nullptr;
        static std::map<std::pair<std::string, std::string>, Eigen::Vector3d> cache;

        if (!model) {
            model = std::make_shared<urdf::Model>();
            std::string urdf_path = "src/alice_xacro_desc/urdf/robot.urdf.xacro";
            
            // Processando arquivo xacro para converter em XML puro URDF
            std::string command = "xacro " + urdf_path;
            FILE* pipe = popen(command.c_str(), "r");
            if (!pipe) {
                std::cerr << "Falha ao executar xacro em: " << urdf_path << std::endl;
                return Eigen::MatrixXd(0, 3);
            }
            char buffer[256];
            std::string urdf_string = "";
            while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
                urdf_string += buffer;
            }
            pclose(pipe);

            if (!model->initString(urdf_string)) {
                std::cerr << "Falha ao inicializar modelo URDF a partir do xacro: " << urdf_path << std::endl;
                return Eigen::MatrixXd(0, 3);
            }
            cache.clear();
        }

        Eigen::MatrixXd result(joint_pairs.size(), 3);
        
        for (size_t i = 0; i < joint_pairs.size(); ++i) {
            const auto& pair = joint_pairs[i];
            
            // Verifica se a medida ja esta no cache
            auto it = cache.find(pair);
            if (it != cache.end()) {
                result.row(i) = it->second;
            } else {
                // Calcula a distancia
                Eigen::Matrix4d T_start = getAbsoluteTransform(model, pair.first);
                Eigen::Matrix4d T_end = getAbsoluteTransform(model, pair.second);
                
                // Distancia relativa (diferenca no sistema de coordenadas da base)
                Eigen::Vector3d pos_start = T_start.block<3,1>(0,3);
                Eigen::Vector3d pos_end = T_end.block<3,1>(0,3);
                
                Eigen::Vector3d diff = pos_end - pos_start;
                
                // Salva no cache
                cache[pair] = diff;
                
                result.row(i) = diff;
            }
        }
        
        return result;
    }

    // Função que recebe os ângulos e retorna a posição final do pé (X, Y, Z)
    Eigen::Vector3d calcularPosicaoPe(double roll_tronco_in, double pitch_tronco_in, double roll_pe_in, double pitch_pe_in, bool use_degrees) {
        // Conversão para radianos se a entrada estiver em graus
        double roll_tronco = use_degrees ? roll_tronco_in * M_PI / 180.0 : roll_tronco_in;
        double pitch_tronco = use_degrees ? pitch_tronco_in * M_PI / 180.0 : pitch_tronco_in;
        double roll_pe = use_degrees ? roll_pe_in * M_PI / 180.0 : roll_pe_in;
        double pitch_pe = use_degrees ? pitch_pe_in * M_PI / 180.0 : pitch_pe_in;

        // Distância entre juntas em milimetros
        Eigen::MatrixXd diff = medirDistanciasJuntas({
            {"base_link_l_up_leg_conn_joint", "l_up_leg_conn_l_up_back_leg_joint"},
            {"l_up_leg_conn_l_up_back_leg_joint", "l_up_back_leg_l_knee_joint"},
            {"l_up_back_leg_l_knee_joint", "l_knee_l_dn_back_leg_joint"},
            {"l_knee_l_dn_back_leg_joint", "l_dn_back_leg_l_ankle_joint"},
            {"l_dn_back_leg_l_ankle_joint", "l_ankle_l_foot_joint"}
        });

               

        // Definição das matrizes de transformação homogênea (4x4)
        Eigen::Matrix4d Rtronco;
        Rtronco << 1, 0, 0, 0,
                    0, std::cos(roll_tronco), -std::sin(roll_tronco), 0,
                    0, std::sin(roll_tronco), std::cos(roll_tronco), diff(0,2),
                    0, 0, 0, 1;

        Eigen::Matrix4d Ptronco;
        Ptronco << std::cos(pitch_tronco), 0, std::sin(pitch_tronco), 0,
                    0, 1, 0, 0,
                    -std::sin(pitch_tronco), 0, std::cos(pitch_tronco), diff(1,2),
                    0, 0, 0, 1;

        Eigen::Matrix4d Pjoelhoup;
        Pjoelhoup << std::cos(-pitch_tronco), 0, std::sin(-pitch_tronco), 0,
                        0, 1, 0, 0,
                        -std::sin(-pitch_tronco), 0, std::cos(-pitch_tronco), diff(2,2),
                        0, 0, 0, 1;

        Eigen::Matrix4d Ppe;
        Ppe << std::cos(pitch_pe), 0, std::sin(pitch_pe), 0,
                0, 1, 0, 0,
                -std::sin(pitch_pe), 0, std::cos(pitch_pe), diff(3,2),
                0, 0, 0, 1;

        Eigen::Matrix4d Ppeup;
        Ppeup << std::cos(-pitch_pe), 0, std::sin(-pitch_pe), 0,
                    0, 1, 0, 0,
                    -std::sin(-pitch_pe), 0, std::cos(-pitch_pe), diff(4,2),
                    0, 0, 0, 1;

        Eigen::Matrix4d Rpe;
        Rpe << 1, 0, 0, 0,
                0, std::cos(roll_pe), -std::sin(roll_pe), 0,
                0, std::sin(roll_pe), std::cos(roll_pe), -0.061,
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