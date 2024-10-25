import serial
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

from time import sleep

from datetime import datetime

ser_pid = serial.Serial(port='COM5', baudrate=9600) 

flag_vel = False
flag_pid = False
flag_temp = False

def save_serial_data(path):
    file = open(path, "w")
    file.write("velocidade,pid,temp") # Colunas
    file.write("\n")

    while True:
        global flag_vel, flag_pid, flag_temp

        data_pid = str(ser_pid.readline())

        if "velocidade" in data_pid:
            _, vel = data_pid.split(":")
            vel = vel[:-5]
            flag_vel = True
        if "pid" in data_pid:
            _, pid = data_pid.split(":")
            pid = pid[:-5]
            flag_pid = True
        if "Temperatura" in data_pid:
            _, temp = data_pid.split(":")
            temp = temp[:-5]
            flag_temp = True
        
        if flag_vel == True and flag_pid == True and flag_temp == True:
            flag_vel = False
            flag_pid = False
            flag_temp = False
            file.write(f"{vel},{pid},{temp}") 
            file.write("\n")
            print(f"Tempo: {datetime.now()} | Velocidade: {vel} | PID: {pid} | Temperatura: {temp}")


def plot_data(path, setpoint):
    df = pd.read_csv(path)
    velocidade = df["velocidade"].to_numpy()
    tempo = np.arange(0, len(velocidade))
    temp = df["temp"].to_numpy()
    plt.plot(tempo, velocidade, label="Velocidade")
    plt.plot(tempo, temp, label="Temperatura")
    plt.legend()
    plt.show()

def init_parameters(setpoint, kp, ki, kd):
    while True:
        is_ready = str(ser_pid.readline())
        if "Ready" in is_ready:
            while True:
                if "setup_completed" in str(ser_pid.readline()):
                    print("Parameters initialized successfully.")
                    return
                ser_pid.write(f"{setpoint}S".encode())
                ser_pid.write(f"{kp}P".encode())
                ser_pid.write(f"{ki}I".encode())
                ser_pid.write(f"{kd}D".encode())

init_parameters("33", "1.1", "0.000008435", "0.0")
save_serial_data("teste_ciclo2.csv")
#plot_data("teste_ciclo2.csv", 35)
