import math
import pandas
import matplotlib.pyplot as plt

def polar_to_cartesian(radius, phi):
    x = radius * math.cos(phi)
    y = radius * math.sin(phi)
    return x, y

# Dados de exemplo
dados = pandas.read_csv('./lidarPointCloud.csv', sep=',', header=None)  # Lê o arquivo CSV sem cabeçalho

# Converter coordenadas polares para cartesianas
x_list = []
y_list = []
for index, row in dados.iterrows():
    theta, phi = row[1], row[2]  # Índices 1 e 2 correspondem à segunda e terceira colunas
    x, y = polar_to_cartesian(theta, phi - 2.55)
    x_list.append(x)
    y_list.append(y)

def plot_lidar_coordinates():
  plt.figure(figsize=(8, 6))
  plt.plot(x_list, y_list, marker='o', linestyle='', markersize=5)
  plt.xlabel('X')
  plt.ylabel('Y')
  plt.title('Plot de Coordenadas Cartesianas')
  plt.grid(True)
  plt.savefig('plot.svg')  # Salvar o plot como um arquivo SVG
  plt.show()

# plot_lidar_coordinates()


