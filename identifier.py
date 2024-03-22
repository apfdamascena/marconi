from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error
from sklearn.metrics import r2_score
from sklearn.pipeline import make_pipeline
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
import pandas as pd
import numpy as np
from threading import Semaphore
class Identifier:

    def __init__(self, path: str, semaphore: Semaphore) -> None:
        self.__read_semaphore = semaphore
        self.__model = None
        self.__scaler = None
        self.__scaler = StandardScaler()

    def run(self):
        print("[MODEL IDENTIFIER] Acquiring permission to read")
        self.__read_semaphore.acquire(blocking=False)
        self.__data = pd.read_csv(path)

        X = self.__data[['Velocidade linear', 'Velocidade angular']].values
        y = self.__data[['Velocidade Roda Esquerda', 'Velocidade Roda Direita']].values

        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

        X_train_scaled = self.__scaler.fit_transform(X_train)
        X_test_scaled = self.__scaler.transform(X_test)

        
        kernel = 1.0 * RBF(length_scale=1.0)
        self.__model = make_pipeline(GaussianProcessRegressor(kernel=kernel, random_state=42))
        self.__model.fit(X_train_scaled, y_train)

        
        y_pred_train = self.__model.predict(X_train_scaled)
        y_pred_test = self.__model.predict(X_test_scaled)

        mse_train = mean_squared_error(y_train, y_pred_train)
        mse_test = mean_squared_error(y_test, y_pred_test)
        r2_train = r2_score(y_train, y_pred_train)
        r2_test = r2_score(y_test, y_pred_test)

        print("MSE (treinamento):", mse_train)
        print("MSE (teste):", mse_test)
        print("R^2 (treinamento):", r2_train)
        print("R^2 (teste):", r2_test)
        
        print("[MODEL IDENTIFIER] Releasing permission to read")
        self.__read_semaphore.release()

    def predict_new_values(self, linear_speed: float, angular_speed: float):
        if self.__model is None:
            print("Modelo não treinado. Por favor, treine o modelo antes de fazer previsões.")
            return None

        scaled_input = np.array([[linear_speed, angular_speed]])
        scaled_input = self.__scaler.transform(scaled_input) 
        predicted_speeds = self.__model.predict(scaled_input)
        return predicted_speeds
        
        

if __name__ == "__main__":
    identifier = Identifier("./direction/robot_data.csv")
    identifier.run()
    predictions = identifier.predict_new_values(0.1220753045082092, -0.129649029170894)
    print(f"predição: ", predictions)
