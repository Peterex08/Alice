import numpy as np

deg_theta = 12.81
deg_phi = 2.96

L = 265
HBb = np.array([0, 0, -50]) # Translação da origem global até a referencial

Bd = np.array([-54.5, 59.12, -7]) # Engaste atuador direito superior
Be = np.array([-54.5, -59.12, -7]) # Engaste atuador esquerdo superior

bd = np.array([24.76, 46.76, -243.5]) # Engaste atuador direito inferior
be = np.array([24.76, -46.76, -243.5]) # Engaste atuador esquerdo inferior

# =========================================================
# AS DISTÂNCIAS INTERNAS DAS JUNTAS (OFFSETS)
# =========================================================
offset_sup = 25.0 # Distância Z do eixo de Roll ao eixo de Pitch (Base)
offset_inf = 18.5 # A distância 'H' do eixo de Roll ao eixo de Pitch (Joelho)

theta = np.radians(deg_theta)
phi = np.radians(deg_phi)

furo_d_reto = HBb + bd
furo_e_reto = HBb + be

Deltapitch = np.array([
    (L * np.sin(theta)), 
    0,  
    L - (L * np.cos(theta))
])

pd_pitch = furo_d_reto + Deltapitch
pe_pitch = furo_e_reto + Deltapitch

Rroll = np.array([
    [1, 0, 0],
    [0, np.cos(phi), -np.sin(phi)],
    [0, np.sin(phi), np.cos(phi)]
])

PD_glob = Rroll @ pd_pitch
PE_glob = Rroll @ pe_pitch

# =========================================================
# CÁLCULO FINAL COM COMPENSAÇÃO DOS DOIS OFFSETS
# =========================================================

# --- Atuador Direito ---
VX_d = PD_glob[0] - Bd[0]
VY_d = PD_glob[1] - Bd[1]
VZ_d = PD_glob[2] - Bd[2]

# Hipotenusa no plano YZ (onde os offsets atuam)
H_YZ_d = np.sqrt(VY_d**2 + VZ_d**2)

# Pitágoras 3D final descontando as duas peças rígidas
Ld = np.sqrt(VX_d**2 + (H_YZ_d - offset_sup - offset_inf)**2)

# --- Atuador Esquerdo ---
VX_e = PE_glob[0] - Be[0]
VY_e = PE_glob[1] - Be[1]
VZ_e = PE_glob[2] - Be[2]

H_YZ_e = np.sqrt(VY_e**2 + VZ_e**2)

Le = np.sqrt(VX_e**2 + (H_YZ_e - offset_sup - offset_inf)**2)

print(Le, Ld)