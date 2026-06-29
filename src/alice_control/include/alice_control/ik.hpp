#pragma once

namespace ik {

    struct IkResult {
        double theta_joelho;
        double phi_joelho;
        double theta_pe;
        double phi_pe;
    };

    /**
     * @brief Resolve a Cinemática Inversa Analítica para a perna do bípede.
     * @param x_alvo Coordenada X global alvo para o Tornozelo.
     * @param y_alvo Coordenada Y global alvo para o Tornozelo.
     * @param z_alvo Coordenada Z global alvo para o Tornozelo.
     * @param use_degrees Define se a saída deve ser em graus (true) ou radianos (false). Padrão é false (rad).
     * @return IkResult Estrutura contendo os ângulos calculados.
     */
    IkResult calc_ik_analitica_perna(double x_alvo, double y_alvo, double z_alvo, bool use_degrees = false);

} // namespace ik
