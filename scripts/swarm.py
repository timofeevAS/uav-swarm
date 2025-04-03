import sqlite3
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import os

# Подключение к базе данных SQLite
conn = sqlite3.connect(os.environ['UAV_DB'])  # Замените на путь к вашей базе данных
cursor = conn.cursor()

# Запрос данных из таблицы swarm_state
cursor.execute("SELECT position_x, position_y, position_z, uav_id FROM swarm_state ORDER BY uav_id, timestamp")
data = cursor.fetchall()

cursor.execute("SELECT target_x, target_y, target_z FROM swarm_state")
targets = cursor.fetchall()

# Закрытие соединения с базой данных
conn.close()

# Разделение данных на координаты и идентификаторы дронов
x = [row[0] for row in data]
y = [row[1] for row in data]
z = [row[2] for row in data]
uav_ids = [row[3] for row in data]

# Создание 3D-графика
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Цвета для каждого дрона
colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']  # Добавьте больше цветов, если дронов больше

# Построение линий для каждого дрона
for uav_id in set(uav_ids):
    # Фильтрация данных для текущего дрона
    x_uav = [x[i] for i in range(len(x)) if uav_ids[i] == uav_id]
    y_uav = [y[i] for i in range(len(y)) if uav_ids[i] == uav_id]
    z_uav = [z[i] for i in range(len(z)) if uav_ids[i] == uav_id]
    
    # Построение линий
    ax.plot(x_uav, y_uav, z_uav, c=colors[uav_id % len(colors)], label=f'UAV {uav_id}', marker='o')


for target in targets:
    ax.plot(target[0],target[1],target[2],c='brown',marker='x')

ax.plot(10,10,10,c='red',marker='x')
# Настройка графика
ax.set_xlabel('Position X')
ax.set_ylabel('Position Y')
ax.set_zlabel('Position Z')
ax.set_title('3D Trajectory of UAVs')
ax.legend()  # Добавление легенды

# Показать график
plt.show()