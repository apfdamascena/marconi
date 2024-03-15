import os
import pandas as pd

class ConcatData:

    def __init__(self, path: str) -> None:
        self.__path = path

    def concat(self):

        data = pd.DataFrame()

        for arquivo in os.listdir(self.__path):
            if arquivo.endswith('.csv'):
                caminho_arquivo = os.path.join(self.__path, arquivo)
                dados_arquivo = pd.read_csv(caminho_arquivo)
                data = pd.concat([data, dados_arquivo])

        data.to_csv('./data/robot_data.csv', index=False)



if __name__ == "__main__":
    data = ConcatData("./data")
    data.concat()