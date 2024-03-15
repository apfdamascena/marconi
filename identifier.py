import pandas as pd
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt

class Identifier:

    def __init__(self, path: str) -> None:
        self.__data = pd.read_csv(path)


    def run(self):
        print("oi")

