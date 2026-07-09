/**
 * @file joint_state_publisher_node.cpp
 * @brief Nó ROS 2 com GUI Qt5 para controle interativo das pernas via IK.
 *
 * Exibe 6 sliders (3 por perna: X, Y, Z) que controlam a posição cartesiana
 * do pé. A cinemática inversa (ParallellIK) calcula os ângulos das juntas em
 * tempo real e publica no tópico /joint_states a 50Hz.
 */

#include <QApplication>
#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QTimer>
#include <QPushButton>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "alice_control/parallel_ik.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

// ============================================================
// ROS 2 NODE
// ============================================================
class JointStatePublisherNode : public rclcpp::Node
{
public:
    JointStatePublisherNode()
    : Node("alice_joint_state_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&JointStatePublisherNode::publish, this));
        initJoints();
        RCLCPP_INFO(this->get_logger(), "Inicializado com %zu juntas.", jv_.size());
    }

    /// Resolve IK para perna esquerda. Retorna ângulos via out_angles se não-nulo.
    bool solveLeft(const Eigen::Vector3d & t, std::vector<double> * out = nullptr) {
        std::vector<double> a;
        if (!ik_.solve(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                       t, Eigen::Quaterniond::Identity(), a)) return false;
        jv_["base_link_l_up_leg_conn_joint"]         = a[0]; // eixo x (roll)
        jv_["l_up_leg_conn_l_up_back_leg_joint"]     = a[1]; // eixo y (pitch)
        jv_["l_up_leg_conn_l_l_up_front_leg_joint"]  = a[1]; // eixo y
        jv_["l_up_leg_conn_l_r_up_front_leg_joint"]  = a[1]; // eixo y
        jv_["l_up_back_leg_l_knee_joint"]            = a[2];
        jv_["l_knee_l_dn_back_leg_joint"]            = a[3]; // eixo y (pitch)
        jv_["l_knee_l_l_dn_front_leg_joint"]         = a[3]; // eixo y
        jv_["l_knee_l_r_dn_front_leg_joint"]         = a[3]; // eixo y
        jv_["l_dn_back_leg_l_ankle_joint"]           = a[4];
        jv_["l_ankle_l_foot_joint"]                  = a[5]; // eixo x (roll)
        if (out) *out = a;
        return true;
    }

    /// Resolve IK para perna direita. Retorna ângulos via out_angles se não-nulo.
    bool solveRight(const Eigen::Vector3d & t, std::vector<double> * out = nullptr) {
        std::vector<double> a;
        if (!ik_.solve(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                       t, Eigen::Quaterniond::Identity(), a)) return false;
        jv_["base_link_r_up_leg_conn_joint"]         = a[0]; // eixo x (roll)
        jv_["r_up_leg_conn_r_up_back_leg_joint"]     = a[1]; // eixo y (pitch)
        jv_["r_up_leg_conn_r_l_up_front_leg_joint"]  = a[1]; // eixo y
        jv_["r_up_leg_conn_r_r_up_front_leg_joint"]  = a[1]; // eixo y
        jv_["r_up_back_leg_r_knee_joint"]            = a[2];
        jv_["r_knee_r_dn_back_leg_joint"]            = a[3]; // eixo y (pitch)
        jv_["r_knee_r_l_dn_front_leg_joint"]         = a[3]; // eixo y
        jv_["r_knee_r_r_dn_front_leg_joint"]         = a[3]; // eixo y
        jv_["r_dn_back_leg_r_ankle_joint"]           = a[4];
        jv_["r_ankle_r_foot_joint"]                  = a[5]; // eixo x (roll)
        if (out) *out = a;
        return true;
    }

private:
    void publish() {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        for (const auto & [n, v] : jv_) { msg.name.push_back(n); msg.position.push_back(v); }
        publisher_->publish(msg);
    }

    void initJoints() {
        // Cabeça (z=yaw, y=pitch)
        jv_["base_link_yaw_head_joint"] = 0.0;
        jv_["yaw_head_pitch_head_joint"] = 0.0;
        // Braço esquerdo (y=pitch, z=prismatic)
        jv_["base_link_l_arm_tri_joint"] = 0.0;
        jv_["l_arm_tri_l_arm_haste_joint"] = 0.0;
        jv_["l_arm_haste_l_arm_atuador_joint"] = 0.0;
        jv_["l_arm_tri_l_arm_ant_joint"] = 0.0;
        // Braço direito
        jv_["base_link_r_arm_tri_joint"] = 0.0;
        jv_["r_arm_tri_r_arm_haste_joint"] = 0.0;
        jv_["r_arm_haste_r_arm_atuador_joint"] = 0.0;
        jv_["r_arm_tri_r_arm_ant_joint"] = 0.0;
        // Perna esquerda central
        jv_["base_link_l_up_leg_conn_joint"] = 0.0;
        jv_["l_up_leg_conn_l_l_up_front_leg_joint"] = 0.0;
        jv_["l_up_leg_conn_l_r_up_front_leg_joint"] = 0.0;
        jv_["l_up_leg_conn_l_up_back_leg_joint"] = 0.0;
        jv_["l_up_back_leg_l_knee_joint"] = 0.0;
        jv_["l_knee_l_l_dn_front_leg_joint"] = 0.0;
        jv_["l_knee_l_dn_back_leg_joint"] = 0.0;
        jv_["l_knee_l_r_dn_front_leg_joint"] = 0.0;
        jv_["l_dn_back_leg_l_ankle_joint"] = 0.0;
        jv_["l_ankle_l_foot_joint"] = 0.0;
        // Perna esquerda atuadores (l_l sup, l_r sup, l_l inf, l_r inf)
        jv_["base_link_l_l_up_up_leg_conn_joint"] = 0.0;
        jv_["l_l_up_up_leg_conn_l_l_up_haste_joint"] = 0.0;
        jv_["l_l_up_haste_l_l_up_atuador_joint"] = 0.0;
        jv_["l_l_up_atuador_l_l_up_dn_leg_conn_joint"] = 0.0;
        jv_["base_link_l_r_up_up_leg_conn_joint"] = 0.0;
        jv_["l_r_up_up_leg_conn_l_r_up_haste_joint"] = 0.0;
        jv_["l_r_up_haste_l_r_up_atuador_joint"] = 0.0;
        jv_["l_r_up_atuador_l_r_up_dn_leg_conn_joint"] = 0.0;
        jv_["l_l_dn_front_leg_l_l_dn_up_leg_conn_joint"] = 0.0;
        jv_["l_l_dn_up_leg_conn_l_l_dn_atuador_joint"] = 0.0;
        jv_["l_l_dn_atuador_l_l_dn_haste_joint"] = 0.0;
        jv_["l_l_dn_haste_l_l_dn_dn_leg_conn_joint"] = 0.0;
        jv_["l_r_dn_front_leg_l_r_dn_up_leg_conn_joint"] = 0.0;
        jv_["l_r_dn_up_leg_conn_l_r_dn_atuador_joint"] = 0.0;
        jv_["l_r_dn_atuador_l_r_dn_haste_joint"] = 0.0;
        jv_["l_r_dn_haste_l_r_dn_dn_leg_conn_joint"] = 0.0;
        // Perna direita central
        jv_["base_link_r_up_leg_conn_joint"] = 0.0;
        jv_["r_up_leg_conn_r_l_up_front_leg_joint"] = 0.0;
        jv_["r_up_leg_conn_r_r_up_front_leg_joint"] = 0.0;
        jv_["r_up_leg_conn_r_up_back_leg_joint"] = 0.0;
        jv_["r_up_back_leg_r_knee_joint"] = 0.0;
        jv_["r_knee_r_l_dn_front_leg_joint"] = 0.0;
        jv_["r_knee_r_dn_back_leg_joint"] = 0.0;
        jv_["r_knee_r_r_dn_front_leg_joint"] = 0.0;
        jv_["r_dn_back_leg_r_ankle_joint"] = 0.0;
        jv_["r_ankle_r_foot_joint"] = 0.0;
        // Perna direita atuadores
        jv_["base_link_r_l_up_up_leg_conn_joint"] = 0.0;
        jv_["r_l_up_up_leg_conn_r_l_up_haste_joint"] = 0.0;
        jv_["r_l_up_haste_r_l_up_atuador_joint"] = 0.0;
        jv_["r_l_up_atuador_r_l_up_dn_leg_conn_joint"] = 0.0;
        jv_["base_link_r_r_up_up_leg_conn_joint"] = 0.0;
        jv_["r_r_up_up_leg_conn_r_r_up_haste_joint"] = 0.0;
        jv_["r_r_up_haste_r_r_up_atuador_joint"] = 0.0;
        jv_["r_r_up_atuador_r_r_up_dn_leg_conn_joint"] = 0.0;
        jv_["r_l_dn_front_leg_r_l_dn_up_leg_conn_joint"] = 0.0;
        jv_["r_l_dn_up_leg_conn_r_l_dn_atuador_joint"] = 0.0;
        jv_["r_l_dn_atuador_r_l_dn_haste_joint"] = 0.0;
        jv_["r_l_dn_haste_r_l_dn_dn_leg_conn_joint"] = 0.0;
        jv_["r_r_dn_front_leg_r_r_dn_up_leg_conn_joint"] = 0.0;
        jv_["r_r_dn_up_leg_conn_r_r_dn_atuador_joint"] = 0.0;
        jv_["r_r_dn_atuador_r_r_dn_haste_joint"] = 0.0;
        jv_["r_r_dn_haste_r_r_dn_dn_leg_conn_joint"] = 0.0;
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::map<std::string, double> jv_;
    parallel_ik::ParallellIK ik_;
};

// ============================================================
// QT5 GUI — 6 sliders para controle XYZ de cada perna
// ============================================================

/// Dados de uma fileira de slider (eixo)
struct AxisRow {
    QSlider * slider;
    QLabel  * valLabel;
};

/// Dados de um grupo de perna (3 eixos + status)
struct LegGroup {
    AxisRow x, y, z;
    QLabel * statusLabel;
    QLabel * anglesLabel;
};

static AxisRow makeAxisRow(QVBoxLayout * parent, const QString & name,
                           int minMM, int maxMM, int defMM)
{
    auto * row = new QHBoxLayout();
    auto * lbl = new QLabel(name);
    lbl->setFixedWidth(28);
    lbl->setStyleSheet("font-weight: bold;");

    auto * minLbl = new QLabel(QString::number(minMM / 1000.0, 'f', 2));
    minLbl->setFixedWidth(45);
    minLbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    minLbl->setStyleSheet("color: #888;");

    auto * slider = new QSlider(Qt::Horizontal);
    slider->setRange(minMM, maxMM);
    slider->setValue(defMM);
    slider->setTickPosition(QSlider::TicksBelow);
    slider->setTickInterval((maxMM - minMM) / 10);

    auto * maxLbl = new QLabel(QString::number(maxMM / 1000.0, 'f', 2));
    maxLbl->setFixedWidth(45);
    maxLbl->setStyleSheet("color: #888;");

    auto * valLbl = new QLabel();
    valLbl->setFixedWidth(75);
    valLbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    valLbl->setStyleSheet("font-family: monospace; font-size: 13px;");
    char buf[32];
    snprintf(buf, sizeof(buf), "%.3f m", defMM / 1000.0);
    valLbl->setText(buf);

    row->addWidget(lbl);
    row->addWidget(minLbl);
    row->addWidget(slider, 1);
    row->addWidget(maxLbl);
    row->addWidget(valLbl);
    parent->addLayout(row);

    // Auto-update value label
    QObject::connect(slider, &QSlider::valueChanged, [valLbl](int v) {
        char b[32];
        snprintf(b, sizeof(b), "%.3f m", v / 1000.0);
        valLbl->setText(b);
    });

    return {slider, valLbl};
}

static LegGroup makeLegGroup(QVBoxLayout * parent, const QString & title)
{
    auto * group = new QGroupBox(title);
    group->setStyleSheet(
        "QGroupBox { font-weight: bold; font-size: 14px; margin-top: 10px; "
        "border: 1px solid #aaa; border-radius: 6px; padding-top: 18px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }");
    auto * lay = new QVBoxLayout(group);
    lay->setSpacing(6);

    AxisRow xr = makeAxisRow(lay, "X:", -200, 200, 0);
    AxisRow yr = makeAxisRow(lay, "Y:", -200, 200, 0);
    AxisRow zr = makeAxisRow(lay, "Z:", -700, -200, -580);

    auto * status = new QLabel("● IK OK");
    status->setStyleSheet("color: #2a2; font-weight: bold; font-size: 12px;");

    auto * angles = new QLabel("roll_t=0.000  pitch_t=0.000  pitch_p=0.000  roll_p=0.000");
    angles->setStyleSheet("font-family: monospace; font-size: 11px; color: #555;");

    lay->addWidget(status);
    lay->addWidget(angles);
    parent->addWidget(group);

    return {xr, yr, zr, status, angles};
}

/// Callback que lê os 3 sliders, chama a IK e atualiza os labels de status
static void onSliderChanged(LegGroup & lg, std::shared_ptr<JointStatePublisherNode> node, bool left)
{
    double x = lg.x.slider->value() / 1000.0;
    double y = lg.y.slider->value() / 1000.0;
    double z = lg.z.slider->value() / 1000.0;

    std::vector<double> a;
    bool ok = left
        ? node->solveLeft(Eigen::Vector3d(x, y, z), &a)
        : node->solveRight(Eigen::Vector3d(x, y, z), &a);

    if (ok) {
        lg.statusLabel->setText("● IK OK");
        lg.statusLabel->setStyleSheet("color: #2a2; font-weight: bold; font-size: 12px;");
        char buf[200];
        snprintf(buf, sizeof(buf),
                 "roll_t=%.3f  pitch_t=%.3f  pitch_p=%.3f  roll_p=%.3f",
                 a[0] * 180.0 / M_PI, a[1] * 180.0 / M_PI,
                 a[3] * 180.0 / M_PI, a[5] * 180.0 / M_PI);
        lg.anglesLabel->setText(buf);
    } else {
        lg.statusLabel->setText("● IK FALHOU");
        lg.statusLabel->setStyleSheet("color: #c22; font-weight: bold; font-size: 12px;");
        lg.anglesLabel->setText("—");
    }
}

// ============================================================
// MAIN
// ============================================================
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    app.setStyle("Fusion");

    auto node = std::make_shared<JointStatePublisherNode>();

    // --- Janela principal ---
    QWidget window;
    window.setWindowTitle("Alice IK Controller");
    window.setMinimumWidth(620);
    window.setStyleSheet("QWidget { font-size: 12px; }");

    auto * mainLay = new QVBoxLayout(&window);
    mainLay->setSpacing(12);

    auto * titleLbl = new QLabel("Alice — Controle IK Interativo");
    titleLbl->setAlignment(Qt::AlignCenter);
    titleLbl->setStyleSheet("font-size: 16px; font-weight: bold; padding: 4px;");
    mainLay->addWidget(titleLbl);

    LegGroup leftLeg  = makeLegGroup(mainLay, "Perna Esquerda");
    LegGroup rightLeg = makeLegGroup(mainLay, "Perna Direita");

    // Reset button
    auto * resetBtn = new QPushButton("Resetar Posições");
    resetBtn->setStyleSheet("padding: 6px 16px;");
    mainLay->addWidget(resetBtn);

    // --- Conectar sliders ---
    auto connectLeg = [&](LegGroup & lg, bool left) {
        auto cb = [&lg, node, left]() { onSliderChanged(lg, node, left); };
        QObject::connect(lg.x.slider, &QSlider::valueChanged, cb);
        QObject::connect(lg.y.slider, &QSlider::valueChanged, cb);
        QObject::connect(lg.z.slider, &QSlider::valueChanged, cb);
    };
    connectLeg(leftLeg, true);
    connectLeg(rightLeg, false);

    // Reset button logic
    QObject::connect(resetBtn, &QPushButton::clicked, [&]() {
        leftLeg.x.slider->setValue(0);
        leftLeg.y.slider->setValue(0);
        leftLeg.z.slider->setValue(-580);
        rightLeg.x.slider->setValue(0);
        rightLeg.y.slider->setValue(0);
        rightLeg.z.slider->setValue(-580);
    });

    // Solve initial positions
    onSliderChanged(leftLeg, node, true);
    onSliderChanged(rightLeg, node, false);

    window.show();

    // --- ROS 2 spin via QTimer ---
    QTimer rosTimer;
    QObject::connect(&rosTimer, &QTimer::timeout, [&node]() {
        rclcpp::spin_some(node);
    });
    rosTimer.start(20); // 50Hz

    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}
