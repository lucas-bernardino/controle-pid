import serial
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

from time import sleep

ser = serial.Serial(port='COM4', baudrate=9600) 

def save_serial_data(path):
    file = open(path, "w")
    flag_vel = False
    flag_pid = False
    file.write("velocidade,pid") # Colunas
    file.write("\n")

    while True:
        data = str(ser.readline())
        if "velocidade" in data:
            _, vel = data.split(":")
            vel = vel[:-5]
            flag_vel = True
        if "pid" in data:
            _, pid = data.split(":")
            pid = pid[:-5]
            flag_pid = True
        if flag_vel == True and flag_pid == True:
            print(vel, pid)
            flag_vel = False
            flag_pid = False
            file.write(f"{vel},{pid}") 
            file.write("\n")

def plot_data(path, setpoint):
    df = pd.read_csv(path)
    velocidade = df["velocidade"].to_numpy()
    erro = [(x - setpoint) for x in velocidade]
    pid = df["pid"].to_numpy()
    tempo = np.arange(0, len(velocidade))
    print(velocidade)
    plt.plot(tempo, velocidade, label="Velocidade")
    plt.plot(tempo, erro, label="Erro")
    plt.plot(tempo, pid, label="Pid")
    plt.yticks(np.arange(min(erro), max(velocidade)))
    plt.legend()
    plt.show()

def init_parameters(setpoint, kp, ki, kd):
    while True:
        is_ready = str(ser.readline())
        if "Ready" in is_ready:
            while True:
                if "setup_completed" in str(ser.readline()):
                    print("Parameters initialized successfully.")
                    return
                ser.write(f"{setpoint}S".encode())
                ser.write(f"{kp}P".encode())
                ser.write(f"{ki}I".encode())
                ser.write(f"{kd}D".encode())
                            

init_parameters("17", "0.5", "0.0", "0.0")
save_serial_data("setpoint_17.csv")
#plot_data("setpoint_20.csv", 20)
