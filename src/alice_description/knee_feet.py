import numpy as np

def calc_actuator_length(deg_theta_joelho=0.0, deg_phi_joelho=0.0, deg_theta_pe=0.0, deg_phi_pe=0.0):
    """
    Calcula a cinemática de uma perna robótica bípede com atuadores paralelos.
    
    Parâmetros:
    - deg_theta_joelho: Ângulo de Pitch (inclinação frontal) da coxa em graus.
    - deg_phi_joelho: Ângulo de Roll (inclinação lateral) global em graus.
    - deg_theta_pe: Ângulo de Pitch (flexão) do joelho/canela em graus.
    - deg_phi_pe: Ângulo de Roll isolado do pé em graus.
    """
    
    # =========================================================
    # 1. PARÂMETROS GERAIS E DA COXA (QUADRIL -> JOELHO)
    # =========================================================
    L_coxa = 265
    HBb = np.array([0, 0, -50]) # Translação da origem global até a base da coxa
    
    Bd_coxa = np.array([-54.5, 59.12, -7]) 
    Be_coxa = np.array([-54.5, -59.12, -7]) 
    bd_coxa_reto = np.array([24.76, 46.76, -243.5]) 
    be_coxa_reto = np.array([24.76, -46.76, -243.5]) 
    
    # OFFSETS DA PARTE SUPERIOR (COXA)
    offset_sup_coxa = 25.0 
    offset_inf_coxa = 18.5 
    
    # Conversão para radianos (necessário para as funções trigonométricas do numpy)
    theta_j = np.radians(deg_theta_joelho)
    phi_j = np.radians(deg_phi_joelho)
    
    # --- Cinemática da Coxa ---
    furo_d_reto = HBb + bd_coxa_reto
    furo_e_reto = HBb + be_coxa_reto
    
    # MATEMÁTICA DO PITCH: 
    # Calcula o deslocamento (Delta) em X e Z como um pêndulo. 
    # X cresce com o seno do ângulo, e Z encurta com o cosseno.
    Deltapitch_j = np.array([
        (L_coxa * np.sin(theta_j)), 
        0,  
        L_coxa - (L_coxa * np.cos(theta_j))
    ])
    
    # MATEMÁTICA DO ROLL:
    # Matriz de rotação padrão em torno do eixo X (Roll).
    # Multiplicar um vetor por essa matriz gira o ponto lateralmente no espaço 3D.
    Rroll_j = np.array([
        [1, 0, 0],
        [0, np.cos(phi_j), -np.sin(phi_j)],
        [0, np.sin(phi_j), np.cos(phi_j)]
    ])
    
    # Posição global dos engastes do joelho (aplica-se a translação do pitch e depois a rotação do roll)
    PD_glob_coxa = Rroll_j @ (furo_d_reto + Deltapitch_j)
    PE_glob_coxa = Rroll_j @ (furo_e_reto + Deltapitch_j)
    
    # Centro do Joelho Global (Fim do mecanismo superior)
    centro_joelho_reto = HBb + np.array([0, 0, -L_coxa])
    centro_joelho_global = Rroll_j @ (centro_joelho_reto + Deltapitch_j)
    
    # =========================================================
    # 2. PARÂMETROS DA CANELA (JOELHO -> PÉ)
    # =========================================================
    L_canela = 265
    HKb = np.array([0, 0, -40]) # Translação da origem do joelho até a base da canela
    
    # Origem inicial da parte de baixo transladada e rotacionada pelo Roll superior
    origem_canela_global = centro_joelho_global + (Rroll_j @ HKb)
        
    Bd_joelho = np.array([15.72, 44.21, -21.5])  
    Be_joelho = np.array([15.72, -44.21, -21.5]) 
    bd_pe_reto = np.array([-44.82, 44.09, -322.85]) 
    be_pe_reto = np.array([-44.82, -44.09, -322.85])
    
    # OFFSETS DA PARTE INFERIOR (CANELA/PÉ)
    offset_sup_canela = 18.5 
    offset_inf_canela = 35.0 
    
    theta_p = np.radians(deg_theta_pe)
    phi_p = np.radians(deg_phi_pe)
    
    # --- Cinemática da Canela ---
    BD_joelho_glob = origem_canela_global + (Rroll_j @ Bd_joelho)
    BE_joelho_glob = origem_canela_global + (Rroll_j @ Be_joelho)
    
    Deltapitch_p = np.array([
        (L_canela * np.sin(theta_p)), 
        0, 
        L_canela - (L_canela * np.cos(theta_p))
    ])
    
    vetor_canela_reto = np.array([0, 0, -L_canela])
    
    # Centro do pé a partir da origem_canela_global
    centro_pe_global = origem_canela_global + Rroll_j @ (vetor_canela_reto + Deltapitch_p)
    
    # Aplicação do Roll isolado do pé sobre seus próprios furos
    Rroll_p = np.array([
        [1, 0, 0],
        [0, np.cos(phi_p), -np.sin(phi_p)],
        [0, np.sin(phi_p), np.cos(phi_p)]
    ])
    
    offset_bd_pe = bd_pe_reto - vetor_canela_reto
    offset_be_pe = be_pe_reto - vetor_canela_reto
    
    # Posição final dos furos inferiores globais
    PD_pe_glob = centro_pe_global + Rroll_j @ (Rroll_p @ offset_bd_pe)
    PE_pe_glob = centro_pe_global + Rroll_j @ (Rroll_p @ offset_be_pe)
    
    # =========================================================
    # 3. ADIÇÃO DA SOLA DO PÉ (TRANSLAÇÃO Z)
    # =========================================================
    Z_sola = 109.76 
    vetor_sola_local = np.array([0, 0, -Z_sola])
    
    # Matriz de rotação em torno do eixo Y (Pitch) para orientar a sola na flexão
    Rpitch_p = np.array([
        [np.cos(theta_p), 0, np.sin(theta_p)],
        [0, 1, 0],
        [-np.sin(theta_p), 0, np.cos(theta_p)]
    ])
    
    # Multiplicação Matricial: Combina as três rotações em cadeia (Roll superior -> Pitch canela -> Roll pé)
    Matriz_Rotacao_Pe = Rroll_j @ Rpitch_p @ Rroll_p
    sola_global = centro_pe_global + (Matriz_Rotacao_Pe @ vetor_sola_local)
    
    # =========================================================
    # 4. FUNÇÃO INTERNA DE CÁLCULO DE DISTÂNCIA
    # =========================================================
    def calcular_atuador(P_inf, P_sup, off_sup, off_inf):
        """
        Calcula a hipotenusa 3D entre dois pontos, descontando as hastes rígidas (offsets) no plano YZ.
        A fórmula interna resolve: L = sqrt(DX^2 + (H_YZ - offsets)^2)
        """
        VX, VY, VZ = P_inf - P_sup
        H_YZ = np.sqrt(VY**2 + VZ**2)
        return np.sqrt(VX**2 + (H_YZ - off_sup - off_inf)**2)
    
    # =========================================================
    # 5. CÁLCULO FINAL DOS ATUADORES
    # =========================================================
    Ld_coxa = calcular_atuador(PD_glob_coxa, Bd_coxa, offset_sup_coxa, offset_inf_coxa)
    Le_coxa = calcular_atuador(PE_glob_coxa, Be_coxa, offset_sup_coxa, offset_inf_coxa)
    
    Ld_canela = calcular_atuador(PD_pe_glob, BD_joelho_glob, offset_sup_canela, offset_inf_canela)
    Le_canela = calcular_atuador(PE_pe_glob, BE_joelho_glob, offset_sup_canela, offset_inf_canela)
    
    # Retornando um dicionário estruturado facilita a integração com APIs ou outras partes do sistema
    return {
        "joelho": {
            "coords_globais": centro_joelho_global.round(2).tolist(),
            "angulos": {"pitch": deg_theta_joelho, "roll": deg_phi_joelho},
            "atuadores": {"direito": round(Ld_coxa, 2), "esquerdo": round(Le_coxa, 2)}
        },
        "pe": {
            "coords_globais": centro_pe_global.round(2).tolist(),
            "angulos": {"pitch": deg_theta_pe, "roll": deg_phi_pe},
            "atuadores": {"direito": round(Ld_canela, 2), "esquerdo": round(Le_canela, 2)}
        },
        "sola": {
            "coords_globais": sola_global.round(2).tolist()
        }
    }

# =========================================================
# TESTE DA FUNÇÃO
# =========================================================
if __name__ == "__main__":
    resultado = calc_actuator_length(deg_theta_joelho=0, deg_phi_joelho=0, deg_theta_pe=0, deg_phi_pe=0)
    
    print("=== DADOS DO JOELHO ===")
    print(f"Coordenadas (X, Y, Z): {resultado['joelho']['coords_globais']}")
    print(f"Atuadores (Dir, Esq):  {resultado['joelho']['atuadores']['direito']} mm, {resultado['joelho']['atuadores']['esquerdo']} mm\n")
    
    print("=== DADOS DO PÉ ===")
    print(f"Coordenadas (X, Y, Z): {resultado['pe']['coords_globais']}")
    print(f"Atuadores (Dir, Esq):  {resultado['pe']['atuadores']['direito']} mm, {resultado['pe']['atuadores']['esquerdo']} mm\n")
    
    print("=== DADOS DA SOLA ===")
    print(f"Coordenadas (X, Y, Z): {resultado['sola']['coords_globais']}")