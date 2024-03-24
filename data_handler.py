import os
import pandas as pd
from robot_hardware import RobotHardware
from threading import Semaphore

class DataHandler:

    def __init__(self, path: str, semaphore: Semaphore) -> None:
        self.__path = path
        self.__read_semaphore = semaphore

    def concat(self, path_pwd: str):
        # print("[DATA HANDLER] Acquiring write permission - Concat")
        # self.__read_semaphore.acquire(blocking=True)

        data = pd.DataFrame()

        for arquivo in os.listdir(path_pwd):
            if arquivo.endswith('.csv'):
                caminho_arquivo = os.path.join(path_pwd, arquivo)
                dados_arquivo = pd.read_csv(caminho_arquivo)
                data = pd.concat([data, dados_arquivo])

        data.to_csv(f'./{path_pwd}/robot_data.csv', index=False)

        # print("[DATA HANDLER] Releasing write permission - Concat")
        # self.__read_semaphore.release(blocking=True)

    def remove_outliers(self):
        # print("[DATA HANDLER] Acquiring write permission - Remove outliers")
        # self.__read_semaphore.acquire(blocking=True)
        
        for file in os.listdir(self.__path):
            if not file.endswith('.csv'):
                continue
        
            pwd = os.path.join(self.__path, file)
            data = pd.read_csv(pwd)
            data = data[data['Velocidade linear'] >= 0]
            description = data.describe()

            Q1 = description.quantile(0.25)
            Q3 = description.quantile(0.75)
            IQR = Q3 - Q1
            lower_bound = Q1 - 1.5 * IQR
            upper_bound = Q3 + 1.5 * IQR

            outliers = (data < lower_bound) | (data > upper_bound)

            data_without_outliers = data[~outliers.any(axis=1)]

            pwd_cleaned = os.path.join(self.__path)

            if not os.path.exists(pwd_cleaned):
                os.makedirs(pwd_cleaned)

            data_without_outliers.to_csv(f'{self.__path}/{file}', index=False)
        print("[DATA HANDLER] Releasing write permission - Remove outliers")
        # self.__read_semaphore.release(blocking=True)


    def add_direction(self, path_pwd: str):

        # print("[DATA HANDLER] Acquiring write permission - Add direction")
        # self.__read_semaphore.acquire(blocking=True)
        
        for file in os.listdir(path_pwd):
            if not file.endswith('.csv'):
                continue
        
            file_info = file.split('_')

            if len(file_info) <= 2:
                continue
            

            left, right = int(file_info[2]), int(file_info[4])

            pwd = os.path.join(path_pwd, file)

            data = pd.read_csv(pwd)

            if left == right:
                data['Direction'] = 1
            elif left < right:
                data['Direction'] = 2
            else:
                data['Direction'] = 3

            pwd_direction = os.path.join("./direction")

            if not os.path.exists(pwd_direction):
                os.makedirs(pwd_direction)

            data.to_csv(f'./direction/{file}', index=False)

        print("[DATA HANDLER] Releasing write permission - Add direction")
        # self.__read_semaphore.release(blocking=True)


if __name__ == "__main__":
    data = DataHandler("./asterix-data", None)
    data.remove_outliers() 
    # data.add_direction("./asterix-data")
    data.concat('./asterix-data')


# 1. theta do lidar 
# 2. corrigir direçao do robo: theta = w*t em que t eh o tempo para percorrer o angulo theta com a velocidade angular w

# t -> sleep

# 3. a gente enviar pro modelo junt com algum V eu acho 
# 4. modelo vai retornar os dois valores da roda
# 5. left, right
# 6. (left / vmax) * 100 -> pwm_left