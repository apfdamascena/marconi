import os
import pandas as pd

class DataHandler:

    def __init__(self, path: str) -> None:
        self.__path = path

    def concat(self, path_pwd: str):

        data = pd.DataFrame()

        for arquivo in os.listdir(path_pwd):
            if arquivo.endswith('.csv'):
                caminho_arquivo = os.path.join(path_pwd, sarquivo)
                dados_arquivo = pd.read_csv(caminho_arquivo)
                data = pd.concat([data, dados_arquivo])

        data.to_csv('./data/robot_data.csv', index=False)


    def remove_outliers(self):

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

            pwd_cleaned = os.path.join("./cleaned-data")

            if not os.path.exists(pwd_cleaned):
                os.makedirs(pwd_cleaned)

            data_without_outliers.to_csv(f'./cleaned-data/{file}.csv', index=False)



if __name__ == "__main__":
    data = DataHandler("./data")
    data.remove_outliers()