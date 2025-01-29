import serial
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from datetime import datetime
import threading

ser_pid = serial.Serial(port='COM4', baudrate=9600, timeout=0.5) 
termopar = serial.Serial("COM6", baudrate=9600)

class BikeState:
    def __init__(self, path):
        self.vel = None
        self.pid = None
        self.t1 = None
        self.t2 = None
        self.t3 = None

        self.previous_temps = [None, None, None]

        self.lock = threading.Lock()

        self.file = open(path, "w")
        self.file.write("velocidade,pid,T1,T2,T3")
        self.file.write("\n")
    
    def clean_state(self):
        self.vel = None
        self.pid = None
        self.t1 = None
        self.t2 = None
        self.t3 = None

    def close_file(self):
        self.file.close()

    def save_to_file(self):
        if self.vel is not None and self.pid is not None and self.t1 is not None and self.t2 is not None and self.t3 is not None:
            with self.lock:
                self.file.write(f"{self.vel},{self.pid},{self.t1},{self.t2},{self.t3}") 
                self.file.write("\n")

                self.print_state_formated()
            
                self.clean_state()

    def update_uno_data(self, new_vel, new_pid):
        with self.lock:
            self.vel = float(new_vel)
            self.pid = float(new_pid)

    def update_nano_data(self, new_t1, new_t2, new_t3):
        with self.lock:
            self.t1 = float(new_t1)
            self.t2 = float(new_t2)
            self.t3 = float(new_t3)

            self.previous_temps = [self.t1, self.t2, self.t3]

    def print_state_formated(self):
        print(f"[INFO] Velocidade: {self.vel} | PID: {self.pid} | T1: {self.t1} | T2: {self.t2} | T3: {self.t3}")

    def get_average_temperature(self):
        with self.lock:
            # TODO: Dont forget to use all three termopars. Im only use these because
            # the first one has connection issues.
            return (self.previous_temps[1] + self.previous_temps[2]) / 2

bikeState = BikeState("apos_esfriar.csv")


def arduino_uno_communication():
    print("I'll start to read")
    count_inutil = 0
    while True:
        try:
            serial_data = ser_pid.read(ser_pid.in_waiting).decode()
            if "INFO" in serial_data:
                serial_data = serial_data.replace("INFO", "").rstrip()
                vel, pid = serial_data.split(",")
                bikeState.update_uno_data(vel, pid)
            elif "end_cycle" in serial_data:
                print("--- FIM DE UM CICLO ---\n\n\n")
                bikeState.update_uno_data(15.0, 0.0)
                current_temp = bikeState.get_average_temperature()

                # TODO: Since this is still a test, remember to check 
                # if current_tempo is below 100 when bluetooth temperature is installed
                # correctly
                count_inutil +=1
                if count_inutil > 20:
                    count_inutil = 0
                    ser_pid.write("continue\n".encode())
        except Exception as e:
            print(f'Leaving: {e}')
            break

def arduino_nano_communication():
    while True:
        raw_data = termopar.readline().decode().rstrip().split(",")
        raw_data = [x.strip() for x in raw_data]
        t1, t2, t3 = float(raw_data[0]), float(raw_data[1]), float(raw_data[2])
        bikeState.update_nano_data(t1, t2, t3)

def global_handler():
    while True:
        bikeState.save_to_file()

current_time = f"{datetime.now()}"
file_name = current_time.split()[1].replace(":", "-").replace(".","")

thread_uno = threading.Thread(target=arduino_uno_communication)
thread_nano = threading.Thread(target=arduino_nano_communication)
thread_file = threading.Thread(target=global_handler)

thread_uno.daemon = True 
thread_nano.daemon = True
thread_file.daemon = True

thread_uno.start()
thread_nano.start()
thread_file.start()

try:
    while True:
        pass
except KeyboardInterrupt:
    bikeState.close_file()
    print("Program terminated.")

