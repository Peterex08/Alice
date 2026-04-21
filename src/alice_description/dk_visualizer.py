import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Configurações iniciais
roll_tronco_init = 0
pitch_tronco_init = 0
roll_pe_init = 0
pitch_pe_init = 0

def calcular_cadeia(r_t, p_t, r_p, p_p):
    # Converte graus para radianos
    rt, pt = np.deg2rad(r_t), np.deg2rad(p_t)
    rp, pp = np.deg2rad(r_p), np.deg2rad(p_p)

    # Matrizes de Transformação conforme sua lógica corrigida
    # Rotação -> depois deslocamento (comprimento da haste)
    Rtronco = np.array([[1, 0, 0, 0], [0, np.cos(rt), -np.sin(rt), 0], [0, np.sin(rt), np.cos(rt), 0], [0, 0, 0, 1]])
    Ptronco = np.array([[np.cos(pt), 0, np.sin(pt), 0], [0, 1, 0, 0], [-np.sin(pt), 0, np.cos(pt), -50], [0, 0, 0, 1]])
    Pjoelhoup = np.array([[np.cos(-pt), 0, np.sin(-pt), 0], [0, 1, 0, 0], [-np.sin(-pt), 0, np.cos(-pt), -265], [0, 0, 0, 1]])
    Ppe = np.array([[np.cos(pp), 0, np.sin(pp), 0], [0, 1, 0, 0], [-np.sin(pp), 0, np.cos(pp), -40], [0, 0, 0, 1]])
    Ppeup = np.array([[np.cos(-pp), 0, np.sin(-pp), 0], [0, 1, 0, 0], [-np.sin(-pp), 0, np.cos(-pp), -265], [0, 0, 0, 1]])
    Rpe = np.array([[1, 0, 0, 0], [0, np.cos(rp), -np.sin(rp), 0], [0, np.sin(rp), np.cos(rp), -50], [0, 0, 0, 1]])
    Tpe = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -60.7], [0, 0, 0, 1]])

    matrices = [Rtronco, Ptronco, Pjoelhoup, Ppe, Ppeup, Rpe, Tpe]
    points = [np.array([0, 0, 0, 1])]
    T_cum = np.eye(4)

    for M in matrices:
        T_cum = T_cum @ M
        points.append(T_cum @ np.array([0, 0, 0, 1]))
    
    return np.array(points)

# Criar a figura e o eixo 3D
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.1, bottom=0.25)

# Plot inicial
pts = calcular_cadeia(roll_tronco_init, pitch_tronco_init, roll_pe_init, pitch_pe_init)
line, = ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], '-o', linewidth=3, color='#3b82f6')

# --- CORREÇÃO DE PROPORÇÃO ---
# Isso impede que as hastes pareçam esticar ao rotacionar
ax.set_box_aspect([1,1,1]) 

# Configurações de eixos
limit = 400
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)
ax.set_zlim(-800, 50)
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')

# Sliders
ax_rt = plt.axes([0.15, 0.15, 0.25, 0.03])
ax_pt = plt.axes([0.15, 0.10, 0.25, 0.03])
ax_rp = plt.axes([0.6, 0.15, 0.25, 0.03])
ax_pp = plt.axes([0.6, 0.10, 0.25, 0.03])

s_rt = Slider(ax_rt, 'Roll Tronco', -90, 90, valinit=roll_tronco_init)
s_pt = Slider(ax_pt, 'Pitch Tronco', -90, 90, valinit=pitch_tronco_init)
s_rp = Slider(ax_rp, 'Roll Pé', -90, 90, valinit=roll_pe_init)
s_pp = Slider(ax_pp, 'Pitch Pé', -90, 90, valinit=pitch_pe_init)

def update(val):
    new_pts = calcular_cadeia(s_rt.val, s_pt.val, s_rp.val, s_pp.val)
    line.set_data(new_pts[:, 0], new_pts[:, 1])
    line.set_3d_properties(new_pts[:, 2])
    fig.canvas.draw_idle()

s_rt.on_changed(update)
s_pt.on_changed(update)
s_rp.on_changed(update)
s_pp.on_changed(update)

plt.show()