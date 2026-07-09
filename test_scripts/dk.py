import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Parâmetros de entrada
roll_tronco_deg = 0
pitch_tronco_deg = 10
roll_pe_deg = 0
pitch_pe_deg = 0

# Conversão para radianos
roll_tronco = np.deg2rad(roll_tronco_deg)
pitch_tronco = np.deg2rad(pitch_tronco_deg)
roll_pe = np.deg2rad(roll_pe_deg)
pitch_pe = np.deg2rad(pitch_pe_deg)

# Definição das matrizes de transformação homogênea
# Cada matriz representa a transformação do frame atual para o próximo
def get_matrices():
    Rtronco = np.array([
        [1, 0, 0, 0],
        [0, np.cos(roll_tronco), -np.sin(roll_tronco), 0],
        [0, np.sin(roll_tronco), np.cos(roll_tronco), -50],
        [0, 0, 0, 1],
    ])

    Ptronco = np.array([
        [np.cos(pitch_tronco), 0, np.sin(pitch_tronco), 0],
        [0, 1, 0, 0],
        [-np.sin(pitch_tronco), 0, np.cos(pitch_tronco), -265],
        [0, 0, 0, 1],
    ])

    Pjoelhoup = np.array([
        [np.cos(-pitch_tronco), 0, np.sin(-pitch_tronco), 0],
        [0, 1, 0, 0],
        [-np.sin(-pitch_tronco), 0, np.cos(-pitch_tronco), -40],
        [0, 0, 0, 1],
    ])

    Ppe = np.array([
        [np.cos(pitch_pe), 0, np.sin(pitch_pe), 0],
        [0, 1, 0, 0],
        [-np.sin(pitch_pe), 0, np.cos(pitch_pe), -265],
        [0, 0, 0, 1],
    ])

    Ppeup = np.array([
        [np.cos(-pitch_pe), 0, np.sin(-pitch_pe), 0],
        [0, 1, 0, 0],
        [-np.sin(-pitch_pe), 0, np.cos(-pitch_pe), -50],
        [0, 0, 0, 1],
    ])

    Rpe = np.array([
        [1, 0, 0, 0],
        [0, np.cos(roll_pe), -np.sin(roll_pe), 0],
        [0, np.sin(roll_pe), np.cos(roll_pe), -61],
        [0, 0, 0, 1],
    ])
    return [Rtronco, Ptronco, Pjoelhoup, Ppe, Ppeup, Rpe]

matrices = get_matrices()
labels = ['Origem', 'Rtronco', 'Ptronco', 'Pjoelhoup', 'Ppe', 'Ppeup', 'Rpe']

# Calculando as posições globais (Cinemática Direta acumulada)
points = [np.array([0, 0, 0, 1])] # Ponto inicial na origem
T_cum = np.eye(4)

for M in matrices:
    T_cum = T_cum @ M
    points.append(T_cum @ np.array([0, 0, 0, 1]))

# Extraindo X, Y, Z para o plot
points = np.array(points)
x, y, z = points[:, 0], points[:, 1], points[:, 2]

# Plot 3D
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Desenha os segmentos da "perna/haste"
ax.plot(x, y, z, '-o', markersize=8, linewidth=3, color='#3b82f6', label='Cadeia Cinemática')

# Adiciona as labels em cada junta
for i, txt in enumerate(labels):
    ax.text(x[i], y[i], z[i], f' {txt}', size=9, zorder=1)

ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_title('Visualização da Cadeia Cinemática Direta')

# Ajuste de escala igual para não distorcer a geometria
max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() / 2.0
mid_x, mid_y, mid_z = (x.max()+x.min())/2, (y.max()+y.min())/2, (z.max()+z.min())/2
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.legend()
plt.show()