import serial
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

from time import sleep

from datetime import datetime

ser_pid = serial.Serial(port='COM4', baudrate=9600) 

def plot_data(path, setpoint):
    df = pd.read_csv(path)
    velocidade = df["velocidade"].to_numpy()
    pid = df["pid"].to_numpy()
    tempo = np.arange(0, len(velocidade))
    plt.plot(tempo, velocidade, label="Velocidade")
    plt.plot(tempo, pid, label="PID")
    plt.plot(tempo, [setpoint for i in range(len(tempo))], label="Setpoint")
    plt.legend()
    plt.show()

def arduino_uno_communication(path):
    file = open(path, "w")
    file.write("velocidade,pid")
    file.write("\n")
    print("I'll start to read")
    while True:
        try:
            serial_data = str(ser_pid.readline().decode())
            if "INFO" in serial_data:
                serial_data = serial_data.replace("INFO", "").rstrip()
                vel, pid = serial_data.split(",")
                print(f"[INFO] Velocidade: {vel} | PID: {pid}")
                file.write(f"{vel},{pid}") 
                file.write("\n")
            else:
                print(serial_data)
        except Exception as e:
            print(f'Leaving: {e}')
            break

current_time = f"{datetime.now()}"
file_name = current_time.split()[1].replace(":", "-").replace(".","")

arduino_uno_communication("teste4.csv")
#plot_data("teste4.csv", 35)
