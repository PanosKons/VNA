import serial
import time
import re
import cmath
import math
import json
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import threading

# ==================== CONFIGURATION ====================
FY6900_PORT = "COM7"
STM32_PORT = "COM5"
BAUDRATE = 115200
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

class VNAState:
    def __init__(self):
        self.lock = threading.Lock()
        self.sweep_freqs = []
        self.sweep_rl = []
        
        self.export_requested = False
        self.export_filename = ""

state = VNAState()

def select_cal_file():
    cal_files = [f for f in os.listdir('.') if f.startswith('cal_') and f.endswith('.json')]
    if not cal_files:
        print("[ERROR] No Calibration files found (cal_*.json).")
        return None
        
    print("\n--- Available Calibration Files ---")
    for i, f in enumerate(cal_files):
        print(f"{i+1}. {f}")
        
    choice = input(f"Select a file (1-{len(cal_files)}) [1]: ").strip()
    try:
        idx = int(choice) - 1 if choice else 0
        return cal_files[idx]
    except (ValueError, IndexError):
        return cal_files[0]

def calculate_gamma(f, M_dut, cal_data):
    str_f = str(f)
    if str_f not in cal_data:
        str_f = str(int(f)) if str(int(f)) in cal_data else str_f
        
    Ed = complex(cal_data[str_f]["Ed"]["real"], cal_data[str_f]["Ed"]["imag"])
    Es = complex(cal_data[str_f]["Es"]["real"], cal_data[str_f]["Es"]["imag"])
    Er = complex(cal_data[str_f]["Er"]["real"], cal_data[str_f]["Er"]["imag"])
    
    gamma = (M_dut - Ed) / (Er + Es * (M_dut - Ed) + 1e-9)
    if abs(gamma) > 1.2: 
        gamma = gamma / abs(gamma)
    return gamma

def export_sweep_to_csv(filename, freqs, rls, prompt_vna=True):
    try:
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Frequency (Hz)", "Return Loss (dB)"])
            for f_hz, rl in zip(freqs, rls):
                writer.writerow([f_hz, rl])
        if prompt_vna:
            print(f"\r\n[SUCCESS] Sweep data successfully exported to {filename}\nVNA> ", end="", flush=True)
        else:
            print(f"\n[SUCCESS] Sweep data successfully exported to {filename}")
    except Exception as e:
        if prompt_vna:
            print(f"\r\n[ERROR] Failed to export to CSV: {e}\nVNA> ", end="", flush=True)
        else:
            print(f"\n[ERROR] Failed to export to CSV: {e}")

def terminal_input_thread():
    print("\n[INFO] Type 'export <filename>' at any time to save the next complete sweep.")
    while True:
        try:
            val = input("VNA> ").strip()
            if not val:
                continue
                
            if val.lower().startswith("export"):
                parts = val.split(" ", 1)
                fname = parts[1].strip() if len(parts) > 1 else "sweep_data.csv"
                if not fname.endswith(".csv"):
                    fname += ".csv"
                    
                with state.lock:
                    state.export_filename = fname
                    state.export_requested = True
                print(f"[INFO] Export scheduled for '{fname}'. Waiting for current sweep to end...")
            else:
                print("[ERROR] Unknown command. Use 'export <filename>'")
        except EOFError:
            break

def hardware_thread_func(awg, ser_stm, freqs_hz, cal_data, num_samples, sweep_mode, upfront_csv):
    while True: 
        for idx, f in enumerate(freqs_hz):
            awg.set_vna_frequencies(f)
            
            M_dut = read_stm32(ser_stm, num_samples)
            gamma = calculate_gamma(f, M_dut, cal_data)
            
            gamma_mag = max(abs(gamma), 1e-5)
            db = 20 * math.log10(gamma_mag)
            
            with state.lock:
                state.sweep_rl[idx] = db
                
            # Export sequence perfectly timed with sweep completion
            if idx == len(freqs_hz) - 1:
                with state.lock:
                    if sweep_mode == "ONE-OFF":
                        if upfront_csv:
                            export_sweep_to_csv(upfront_csv, state.sweep_freqs, state.sweep_rl, prompt_vna=False)
                    elif state.export_requested:
                        export_sweep_to_csv(state.export_filename, state.sweep_freqs, state.sweep_rl, prompt_vna=True)
                        state.export_requested = False
        
        if sweep_mode == "ONE-OFF":
            print("\n[INFO] One-off sweep complete. Hardware thread stopping. Plot remains active.")
            break

def setup_gui(is_log, freqs_mhz):
    fig = plt.figure(figsize=(10, 6))
    fig.canvas.manager.set_window_title("Real-Time VNA - Cartesian Mode")
    fig.patch.set_facecolor('#eaeaea')
    
    ax = fig.add_subplot(111)
    ax.set_title("VNA S11 Return Loss", fontsize=16, fontweight='bold', y=1.02)
    ax.set_facecolor('#fdfdfd')
    ax.set_xlabel("Frequency (MHz)", fontweight="bold")
    ax.set_ylabel("Magnitude (dB)", fontweight="bold")
    ax.grid(True, which="both", linestyle="--", alpha=0.6)
    
    if is_log:
        ax.set_xscale('log')
        
    ax.set_xlim(min(freqs_mhz), max(freqs_mhz))
    
    ax.axhline(-10, color='r', linestyle='--', label='-10 dB (Good Match)')
    
    plot_line, = ax.plot(freqs_mhz, [np.nan]*len(freqs_mhz), 'b.-', linewidth=1.5, markersize=5, label="Return Loss (dB)")
    
    ax.legend(loc="best", facecolor='#ffffff', framealpha=0.9)
    plt.tight_layout()
        
    return fig, ax, plot_line

def main():
    print("=== VNA CARTESIAN SWEEP (RETURN LOSS) ===")
    
    cal_file = select_cal_file()
    if not cal_file: return
    
    try:
        num_samples = int(input("\nSamples per point during measurement 20: ").strip() or "20")
    except ValueError:
        num_samples = 20

    print("\nSweep Mode:")
    print("1. Continuous")
    print("2. One-off")
    choice = input("Select (1 or 2) [1]: ").strip()
    sweep_mode = "ONE-OFF" if choice == '2' else "CONTINUOUS"

    csv_filename = ""
    if sweep_mode == "ONE-OFF":
        csv_filename = input("Enter filename to save CSV data (leave blank to skip saving): ").strip()
        if csv_filename and not csv_filename.endswith(".csv"):
            csv_filename += ".csv"

    with open(cal_file, 'r') as f:
        full_data = json.load(f)
        
    cal_data = full_data["data"]
    is_log = full_data.get("metadata", {}).get("sweep_type") == "log"
    freqs_hz = sorted([float(k) for k in cal_data.keys()])
    freqs_mhz = [f / 1e6 for f in freqs_hz]
    
    with state.lock:
        state.sweep_freqs = freqs_hz
        state.sweep_rl = [np.nan] * len(freqs_hz)
    
    try:
        awg = FY6900(FY6900_PORT, BAUDRATE)
        awg.setup_sine_dual(amplitude_vpp=2.0, offset_v=0.0)
        ser_stm = serial.Serial(STM32_PORT, BAUDRATE, timeout=1)
        time.sleep(2)
        
        hw_thread = threading.Thread(
            target=hardware_thread_func, 
            args=(awg, ser_stm, freqs_hz, cal_data, num_samples, sweep_mode, csv_filename), 
            daemon=True
        )
        hw_thread.start()
        
        if sweep_mode == "CONTINUOUS":
            input_thread = threading.Thread(target=terminal_input_thread, daemon=True)
            input_thread.start()

        fig, ax, plot_line = setup_gui(is_log, freqs_mhz)
        
        plt.ion()
        plt.show(block=False)
        
        while plt.fignum_exists(fig.number):
            with state.lock:
                rl_data = list(state.sweep_rl)
            
            if not np.isnan(rl_data[0]):
                plot_line.set_data(freqs_mhz, rl_data)
                ax.relim()
                ax.autoscale_view(scalex=True, scaley=True)

                y_min = min(-30, np.nanmin(rl_data) - 2)
                y_max = max(-10, np.nanmax(rl_data) + 2)
                ax.set_ylim(y_min, y_max)
            
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(0.1) 
            
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        if 'awg' in locals(): awg.close()
        if 'ser_stm' in locals() and ser_stm.is_open: ser_stm.close()

if __name__ == "__main__":
    main()