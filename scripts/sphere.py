import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Параметры
N = 20         # количество дронов
r = 2        # радиус безопасности одного дрона
R_c = np.array([0, 0, 0])  # центр роя
rho = 0.6

positions = []
layers = []
layer_radius = 2 * r  # минимальный радиус первой сферы

def fibonacci_sphere(samples, radius, center):
    points = []
    offset = 2.0 / samples
    increment = np.pi * (3.0 - np.sqrt(5.0))

    for i in range(samples):
        y = ((i * offset) - 1) + (offset / 2)
        r_xy = np.sqrt(1 - y * y)
        phi = i * increment

        x = np.cos(phi) * r_xy
        z = np.sin(phi) * r_xy
        point = center + radius * np.array([x, y, z])
        points.append(point)
    return points 

while len(positions) < N:
    remaining = N - len(positions)
    num_on_layer = int(4 * np.pi * (layer_radius ** 2) / (4 * r ** 2))  # приблизительно сколько дронов на слое
    num_to_place = min(remaining, num_on_layer)
    new_points = fibonacci_sphere(num_to_place, layer_radius, R_c)

    # проверка на пересечения
    for p in new_points:
        if all(np.linalg.norm(p - q) >= 2 * r for q in positions):
            positions.append(p)
            layers.append(layer_radius)
            if len(positions) >= N:
                break

    layer_radius += 2 * r  # следующий слой

positions = np.array(positions) * (1/rho)
unique_layers = (np.array(sorted(set(layers)))) * (1/rho) + r
layer_radius = max(unique_layers)


distances = np.zeros((N, N))
for i in range(N):
    for j in range(N):
        distances[i, j] = np.linalg.norm(positions[i] - positions[j])

print(distances)

# Визуализация
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Отображаем сферы
u, v = np.mgrid[0:2*np.pi:30j, 0:np.pi:15j]
real_layers = np.array(unique_layers) + r
for rad in real_layers:
    x = rad * np.cos(u) * np.sin(v) + R_c[0]
    y = rad * np.sin(u) * np.sin(v) + R_c[1]
    z = rad * np.cos(v) + R_c[2]
    ax.plot_surface(x, y, z, color='gray', alpha=0.1, linewidth=0)

# Дроны и центр
ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c='blue', s=30, label='Дроны')
ax.scatter(*R_c, c='red', s=50, label='Центр')

for pt in positions:
    # Параметрическое уравнение для сферы
    x = r * np.cos(u) * np.sin(v) + pt[0]
    y = r * np.sin(u) * np.sin(v) + pt[1]
    z = r * np.cos(v) + pt[2]
    
    # Рисуем сферу красного цвета с прозрачностью
    ax.plot_surface(x, y, z, color='red', alpha=0.3, linewidth=0)
 
ax.set_title(f'Упаковка по оболочкам (N={N})')
ax.legend()
ax.set_xlim([-layer_radius, layer_radius])
ax.set_ylim([-layer_radius, layer_radius])
ax.set_zlim([-layer_radius, layer_radius])
plt.tight_layout()
plt.show()