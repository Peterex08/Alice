import numpy as np

def calc_ik_analitica_perna(x_alvo, y_alvo, z_alvo):
    """
    Resolve a Cinemática Inversa Analítica para a perna do bípede.
    
    Parâmetros:
    - x_alvo, y_alvo, z_alvo: Coordenadas globais desejadas para o Tornozelo (centro do pé).
    
    Retorna:
    - Dicionário com os ângulos exatos em graus.
    """
    
    # 1. PARÂMETROS FIXOS DA GEOMETRIA (Extraídos da Cinemática Direta)
    L_coxa = 265.0
    L_canela = 265.0
    HBb_z = -50.0       # Offset Z da origem global até a base da coxa
    offset_joelho_z = -40.0 # Offset Z entre o fim da coxa e o início da canela (HKb)
    
    # 2. ISOLAR O VETOR DA PERNA (Removendo o offset do quadril)
    X_v = x_alvo
    Y_v = y_alvo
    Z_v = z_alvo - HBb_z
    
    # 3. SOLVER DO ROLL DO QUADRIL (phi_joelho)
    # Qual o ângulo lateral necessário para alinhar a perna com o alvo no eixo Y?
    phi_j_rad = np.arctan2(Y_v, -Z_v)
    deg_phi_joelho = np.degrees(phi_j_rad)
    
    # 4. DESENROLAR PARA O PLANO 2D (Sagital)
    # Ao "desfazer" o roll, o Y se torna 0 e a perna vira um mecanismo 2D
    X_2d = X_v
    Z_2d = -np.sqrt(Y_v**2 + Z_v**2) # A hipotenusa no plano YZ. Negativo porque aponta para baixo.
    
    # Compensar o pequeno degrau de 40mm do joelho no plano Z
    Z_2d_eff = Z_2d - offset_joelho_z
    
    # 5. SOLVER DO PITCH (Plano 2D - Lei dos Cossenos)
    # Distância reta do quadril até o tornozelo no plano 2D
    D_quadrado = X_2d**2 + Z_2d_eff**2
    
    # Verificação de segurança (Singularidade / Fora de alcance)
    L_max = (L_coxa + L_canela)**2
    L_min = (L_coxa - L_canela)**2
    if D_quadrado > L_max or D_quadrado < L_min:
        raise ValueError("O ponto alvo está fora do alcance físico da perna.")
        
    # C é o cosseno do ângulo interno formado pela dobra do joelho
    C = (D_quadrado - L_coxa**2 - L_canela**2) / (2 * L_coxa * L_canela)
    C = np.clip(C, -1.0, 1.0) # Prevenção contra erros de precisão do float (ex: 1.000000002)
    
    # alpha é a diferença relativa entre o ângulo da coxa e da canela
    alpha = np.arccos(C) 
    
    # Variáveis auxiliares para o sistema linear geométrico
    A = L_coxa + L_canela * np.cos(alpha)
    B = L_canela * np.sin(alpha)
    
    # Resolver o Pitch da Coxa (theta_joelho)
    sin_theta_j = (A * X_2d - B * Z_2d_eff) / D_quadrado
    cos_theta_j = (-B * X_2d - A * Z_2d_eff) / D_quadrado
    theta_j_rad = np.arctan2(sin_theta_j, cos_theta_j)
    
    # Resolver o Pitch da Canela (theta_pe)
    # Na sua arquitetura, theta_p é absoluto, então basta subtrair alpha do quadril
    theta_p_rad = theta_j_rad - alpha
    
    deg_theta_joelho = np.degrees(theta_j_rad)
    deg_theta_pe = np.degrees(theta_p_rad)
    
    # 6. MANTER O PÉ RETO
    # Para que a sola do robô fique perfeitamente paralela ao chão lateralmente,
    # o Roll do tornozelo deve ser exatamente o inverso do Roll do quadril.
    deg_phi_pe = -deg_phi_joelho
    
    return {
        "deg_theta_joelho": round(deg_theta_joelho, 2),
        "deg_phi_joelho": round(deg_phi_joelho, 2),
        "deg_theta_pe": round(deg_theta_pe, 2),
        "deg_phi_pe": round(deg_phi_pe, 2)
    }

# ==========================================
# TESTE DO ALGORITMO
# ==========================================
if __name__ == "__main__":
    # Teste 1: Perna perfeitamente esticada para baixo
    # Z = -50 (base) - 265 (coxa) - 40 (joelho) - 265 (canela) = -620
    angulos = calc_ik_analitica_perna(x_alvo=0, y_alvo=0, z_alvo=-620)
    print("Teste Perna Reta:", angulos) 
    # Esperado: Tudo 0.0

    # Teste 2: Passo para frente e levemente para a esquerda
    angulos = calc_ik_analitica_perna(x_alvo=100, y_alvo=50, z_alvo=-580)
    print("Teste Passo:", angulos)