import serial
import time
import re
import cmath
import math
import json
import numpy as np
import os

# ==================== CONFIGURATION ====================
FY6900_PORT = "COM7"
STM32_PORT = "COM5"
BAUDRATE = 115200

FREQ_START = 1_000_000
FREQ_STOP  = 20_000_000
IF_OFFSET = 10000.0
# =======================================================

class FY6900:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=1.0)
        time.sleep(0.5)
        self.ser.reset_input_buffer()

    def send(self, cmd: str):
        self.ser.reset_input_buffer()
        self.ser.write(f"{cmd}\n".encode("ascii"))
        self.ser.readline()

    def setup_sine_dual(self, amplitude_vpp: float, offset_v: float = 0.0):
        self.send("WMN0")
        self.send("WFN0")
        self.send("WMW00")
        self.send(f"WMA{amplitude_vpp:.2f}")
        self.send(f"WMO{offset_v:.2f}")
        self.send("WFW00")
        self.send(f"WFA{amplitude_vpp:.2f}")
        self.send(f"WFO{offset_v:.2f}")
        self.send("WMN1")
        self.send("WFN1")

    def set_vna_frequencies(self, f_rf_hz: float):
        f_lo_hz = f_rf_hz - IF_OFFSET
        self.send(f"WMF{f_rf_hz:.6f}")
        self.send(f"WFF{f_lo_hz:.6f}")
        time.sleep(0.05)

    def close(self):
        self.send("WMN0")
        self.send("WFN0")
        self.ser.close()

def read_stm32(ser_stm, samples):
    sum_ratio = complex(0, 0)
    count = 0

    ser_stm.reset_input_buffer()
    
    while count < samples:
        ser_stm.write(b'M')
        line = ser_stm.readline().decode('utf-8', errors='ignore').strip()
        
        match = re.search(r"Ia:\s*([\-0-9\.]+)\s*Qa:\s*([\-0-9\.]+)\s*Ib:\s*([\-0-9\.]+)\s*Qb:\s*([\-0-9\.]+)", line)
        if match:
            ia, qa, ib, qb = map(float, match.groups())
            Va = complex(ia, qa)
            Vb = complex(ib, qb)
            
            if abs(Va) >= 1.0:
                sum_ratio += (Vb / Va)
                count += 1
                
    return sum_ratio / samples

def perform_sweep_measure(awg, ser_stm, freqs, num_samples):
    measurements = {}
    for f in freqs:
        awg.set_vna_frequencies(f)
        measurements[f] = read_stm32(ser_stm, num_samples)
        print(f"  [+] Measured: {f/1e6:5.2f} MHz")
    return measurements

def get_user_settings():
    print("--- Calibration Settings ---")
    
    sweep_type = input("Sweep Type (1: Linear, 2: Logarithmic) [1]: ").strip()
    sweep_type = 'log' if sweep_type == '2' else 'linear'
    
    try:
        num_points = int(input("Number of Points [20]: ").strip() or "20")
    except ValueError:
        num_points = 20
        
    try:
        num_samples = int(input("Samples per point [20]: ").strip() or "20")
    except ValueError:
        num_samples = 20
        
    return sweep_type, num_points, num_samples

def generate_frequencies(sweep_type, num_points):
    if sweep_type == 'linear':
        return list(np.linspace(FREQ_START, FREQ_STOP, num_points))
    else:
        return list(np.logspace(np.log10(FREQ_START), np.log10(FREQ_STOP), num_points))

def main():
    print("=== BROADBAND SOL CALIBRATION ===")
    
    sweep_type, num_points, num_samples = get_user_settings()
    freqs = generate_frequencies(sweep_type, num_points)
    
    cal_file_name = f"cal_{sweep_type}_{num_points}pts.json"
    
    print(f"\n[INFO] Will create file: {cal_file_name}")
    print(f"[INFO] Frequencies: {num_points} | Samples/pt: {num_samples} | Type: {sweep_type.upper()}")
    
    try:
        awg = FY6900(FY6900_PORT, BAUDRATE)
        awg.setup_sine_dual(amplitude_vpp=2.0, offset_v=0.0)
        
        ser_stm = serial.Serial(STM32_PORT, BAUDRATE, timeout=1)
        time.sleep(2)
        
        input("\nStep 1: Leave Node B OPEN and press Enter...")
        M_open = perform_sweep_measure(awg, ser_stm, freqs, num_samples)
        
        input("\nStep 2: SHORT Node B and press Enter...")
        M_short = perform_sweep_measure(awg, ser_stm, freqs, num_samples)
        
        input("\nStep 3: Connect 50 OHM LOAD to Node B and press Enter...")
        M_load = perform_sweep_measure(awg, ser_stm, freqs, num_samples)
        
        cal_data = {
            "metadata": {
                "sweep_type": sweep_type,
                "num_points": num_points,
                "f_start": FREQ_START,
                "f_stop": FREQ_STOP
            },
            "data": {}
        }
        
        for f in freqs:
            Ed = M_load[f]
            den = M_open[f] - M_short[f]
            if abs(den) < 1e-9: den = 1e-9
            
            Es = (M_open[f] + M_short[f] - 2.0 * Ed) / den
            Er = (M_open[f] - Ed) * (1.0 - Es)
            
            cal_data["data"][str(f)] = {
                "Ed": {"real": Ed.real, "imag": Ed.imag},
                "Es": {"real": Es.real, "imag": Es.imag},
                "Er": {"real": Er.real, "imag": Er.imag}
            }
            
        with open(cal_file_name, 'w') as f_out:
            json.dump(cal_data, f_out, indent=4)
            
        print(f"\n[SUCCESS] Calibration saved to {cal_file_name}")
        
    finally:
        if 'awg' in locals(): awg.close()
        if 'ser_stm' in locals() and ser_stm.is_open: ser_stm.close()

if __name__ == "__main__":
    main()