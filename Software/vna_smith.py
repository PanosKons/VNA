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
import matplotlib.gridspec as gridspec
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
        self.target_freq_mhz = 1.0
        self.sweep_freqs = []
        self.sweep_real = []
        self.sweep_imag = []
        self.sweep_gammas = []
        
        self.single_f_current = 0.0
        self.single_gamma_current = complex(1, 0)
        
        self.export_requested = False
        self.export_filename = ""

state = VNAState()

def setup_gui(mode):
    fig = plt.figure(figsize=(12, 10))
    fig.canvas.manager.set_window_title(f"Real-Time VNA - {mode} Mode")
    fig.patch.set_facecolor('#eaeaea')
    
    gs = gridspec.GridSpec(2, 1, height_ratios=[2.5, 1])
    ax_smith = fig.add_subplot(gs[0])
    ax_table = fig.add_subplot(gs[1])
    
    ax_smith.set_title("VNA Smith Chart", fontsize=16, fontweight='bold', y=0.95)
    ax_smith.set_facecolor('#fdfdfd')
    ax_smith.set_aspect('equal')
    ax_smith.axis('off')
    ax_smith.set_xlim(-1.2, 1.2)
    ax_smith.set_ylim(-1.2, 1.2)

    theta = np.linspace(0, 2*np.pi, 200)
    ax_smith.plot(np.cos(theta), np.sin(theta), 'k-', lw=1.5)
    ax_smith.plot([-1, 1], [0, 0], 'k-', lw=0.5)

    for r in [0.2, 0.5, 1.0, 2.0, 5.0]:
        c, rad = r/(r+1), 1/(r+1)
        ax_smith.plot(c + rad * np.cos(theta), rad * np.sin(theta), 'gray', lw=0.5, ls='--')

    unit_circle = plt.Circle((0, 0), 1, transform=ax_smith.transData)
    for x in [0.2, 0.5, 1.0, 2.0, 5.0, -0.2, -0.5, -1.0, -2.0, -5.0]:
        cx, cy, rad = 1.0, 1.0/x, abs(1.0/x)
        arc = ax_smith.plot(cx + rad * np.cos(theta), cy + rad * np.sin(theta), 'gray', lw=0.5, ls='--')[0]
        arc.set_clip_path(unit_circle)

    locus_line, = ax_smith.plot([], [], 'b-', linewidth=2, marker='.', markersize=4, label='Trace')
    start_point, = ax_smith.plot([], [], 'go', markersize=8, label='Start Frequency')
    stop_point, = ax_smith.plot([], [], 'ro', markersize=8, label='Stop Frequency')
    
    if mode == "SWEEP":
        ax_smith.legend(loc='upper right', bbox_to_anchor=(1.15, 1.05), fontsize=10, facecolor='#ffffff', framealpha=0.9)

    ax_table.axis('off')
    table_text = ax_table.text(0.5, 0.5, '', ha='center', va='center', 
                               fontfamily='monospace', fontsize=10, 
                               bbox=dict(facecolor='#ffffff', alpha=0.9, edgecolor='#cccccc', boxstyle='round,pad=1'))

    plt.tight_layout()
    return fig, ax_smith, locus_line, start_point, stop_point, table_text

def generate_table_str(freqs, gammas):
    header = f" {'Freq (MHz)':^11} | {'|Γ|':^5} | {'Phase(°)':^8} | {'Z (Ω)':^22} "
    sep = "-" * len(header)
    
    rows = []
    for f, g in zip(freqs, gammas):
        if abs(1 - g) < 1e-5:
            z_str = "Infinity (Open)"
        else:
            z = 50 * (1 + g)/(1 - g)
            sign = '+' if z.imag >= 0 else '-'
            z_str = f"{z.real:7.1f} {sign} j{abs(z.imag):<6.1f}"
            
        ph = math.degrees(cmath.phase(g))
        rows.append(f" {f/1e6:11.2f} | {abs(g):5.2f} | {ph:8.1f} | {z_str:^22} ")
        
    if len(rows) <= 12:
        return f"{header}\n{sep}\n" + "\n".join(rows)
    else:
        half = math.ceil(len(rows) / 2)
        out = f"{header}   ||   {header}\n{sep}   ||   {sep}\n"
        for i in range(half):
            left = rows[i]
            right = rows[i + half] if (i + half) < len(rows) else " " * len(left)
            out += f"{left}   ||   {right}\n"
        return out

def export_sweep_to_csv(filename, freqs, gammas, prompt_vna=True):
    try:
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Frequency (Hz)", "Gamma Real", "Gamma Imag", "Magnitude", "Phase (deg)", "Impedance Real (Ohm)", "Impedance Imag (Ohm)"])
            for f_hz, g in zip(freqs, gammas):
                mag = abs(g)
                phase = math.degrees(cmath.phase(g))
                if abs(1 - g) < 1e-5:
                    z_real, z_imag = float('inf'), 0.0
                else:
                    z = 50 * (1 + g)/(1 - g)
                    z_real, z_imag = z.real, z.imag
                writer.writerow([f_hz, g.real, g.imag, mag, phase, z_real, z_imag])
        
        if prompt_vna:
            print(f"\r\n[SUCCESS] Sweep data successfully exported to {filename}\nVNA> ", end="", flush=True)
        else:
            print(f"\n[SUCCESS] Sweep data successfully exported to {filename}")
    except Exception as e:
        if prompt_vna:
            print(f"\r\n[ERROR] Failed to export to CSV: {e}\nVNA> ", end="", flush=True)
        else:
            print(f"\n[ERROR] Failed to export to CSV: {e}")

def terminal_input_thread(mode):
    if mode == 'SINGLE':
        print("\n[INFO] Type a new frequency (MHz) at any time and press Enter.")
    elif mode == 'SWEEP':
        print("\n[INFO] Type 'export <filename>' at any time to save the next complete sweep.")
        
    while True:
        try:
            val = input("VNA> ").strip()
            if not val:
                continue
                
            if mode == 'SINGLE':
                try:
                    with state.lock:
                        state.target_freq_mhz = float(val)
                except ValueError:
                    print("[ERROR] Invalid frequency format.")
                    
            elif mode == 'SWEEP':
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

def get_nearest_cal_freq(target_hz, available_freqs):
    return min(available_freqs, key=lambda x: abs(x - target_hz))

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

def hardware_thread_func(mode, awg, ser_stm, freqs_hz, cal_data, num_samples, sweep_mode, upfront_csv):
    if mode == 'SINGLE':
        current_f = None
        while True:
            with state.lock:
                target_hz = state.target_freq_mhz * 1e6
                
            f = get_nearest_cal_freq(target_hz, freqs_hz)
            
            if f != current_f:
                awg.set_vna_frequencies(f)
                current_f = f
                ser_stm.reset_input_buffer()
            
            M_dut = read_stm32(ser_stm, num_samples)
            gamma = calculate_gamma(f, M_dut, cal_data)
            
            with state.lock:
                state.single_f_current = f
                state.single_gamma_current = gamma
                
    elif mode == 'SWEEP':
        while True: 
            for idx, f in enumerate(freqs_hz):
                awg.set_vna_frequencies(f)
                
                M_dut = read_stm32(ser_stm, num_samples)
                gamma = calculate_gamma(f, M_dut, cal_data)
                
                with state.lock:
                    state.sweep_gammas[idx] = gamma
                    state.sweep_real[idx] = gamma.real
                    state.sweep_imag[idx] = gamma.imag
                    
                if idx == len(freqs_hz) - 1:
                    with state.lock:
                        if sweep_mode == "ONE-OFF":
                            if upfront_csv:
                                export_sweep_to_csv(upfront_csv, state.sweep_freqs, state.sweep_gammas, prompt_vna=False)
                        elif state.export_requested:
                            export_sweep_to_csv(state.export_filename, state.sweep_freqs, state.sweep_gammas, prompt_vna=True)
                            state.export_requested = False
            
            if sweep_mode == "ONE-OFF":
                print("\n[INFO] One-off sweep complete. Hardware thread stopping. Plot remains active.")
                break

def main():
    print("=== SMITH CHART VNA ===")
    
    cal_file = select_cal_file()
    if not cal_file: return
    
    try:
        num_samples = int(input("\nSamples per point during measurement [20]: ").strip() or "20")
    except ValueError:
        num_samples = 20
        
    print("\n1. Single Frequency Mode")
    print("2. Broadband Sweep Mode")
    choice = input("Select Mode (1 or 2): ").strip()
    mode = "SINGLE" if choice == '1' else "SWEEP"
    
    sweep_mode = "CONTINUOUS"
    csv_filename = ""
    
    if mode == "SWEEP":
        print("\nSweep Mode:")
        print("1. Continuous")
        print("2. One-off")
        sm_choice = input("Select (1 or 2) [1]: ").strip()
        if sm_choice == '2':
            sweep_mode = "ONE-OFF"
            csv_filename = input("Enter filename to save CSV data (leave blank to skip saving): ").strip()
            if csv_filename and not csv_filename.endswith(".csv"):
                csv_filename += ".csv"
    
    with open(cal_file, 'r') as f:
        full_data = json.load(f)
        
    cal_data = full_data["data"]
    freqs_hz = sorted([float(k) for k in cal_data.keys()])
    
    with state.lock:
        state.sweep_freqs = freqs_hz
        state.sweep_real = [np.nan] * len(freqs_hz)
        state.sweep_imag = [np.nan] * len(freqs_hz)
        state.sweep_gammas = [complex(1, 0)] * len(freqs_hz)
    
    try:
        awg = FY6900(FY6900_PORT, BAUDRATE)
        awg.setup_sine_dual(amplitude_vpp=2.0, offset_v=0.0)
        ser_stm = serial.Serial(STM32_PORT, BAUDRATE, timeout=1)
        time.sleep(2)
        
        hw_thread = threading.Thread(
            target=hardware_thread_func, 
            args=(mode, awg, ser_stm, freqs_hz, cal_data, num_samples, sweep_mode, csv_filename), 
            daemon=True
        )
        hw_thread.start()
        
        if mode == "SINGLE" or (mode == "SWEEP" and sweep_mode == "CONTINUOUS"):
            input_thread = threading.Thread(target=terminal_input_thread, args=(mode,), daemon=True)
            input_thread.start()

        if mode == "SINGLE":
            fig, ax, locus_line, start_point, stop_point, table_text = setup_gui("SINGLE")
            table_text.set_fontsize(14)
            
            plt.ion()
            plt.show(block=False)
            
            while plt.fignum_exists(fig.number):
                with state.lock:
                    gamma = state.single_gamma_current
                    f = state.single_f_current
                
                if abs(1.0 - gamma) < 1e-5:
                    z_str = "Z = Infinity (Open)"
                else:
                    Z_dut = 50.0 * (1.0 + gamma) / (1.0 - gamma)
                    sign = '+' if Z_dut.imag >= 0 else '-'
                    z_str = f"Z = {Z_dut.real:.1f} {sign} j{abs(Z_dut.imag):.1f} Ω"
                
                locus_line.set_data([gamma.real], [gamma.imag])
                locus_line.set_marker('o')
                locus_line.set_markersize(10)
                locus_line.set_color('red')
                
                t_str = f"Frequency: {f/1e6:.2f} MHz\n\n|Γ| = {abs(gamma):.3f}   |   Phase = {math.degrees(cmath.phase(gamma)):.1f}°\n\n{z_str}"
                table_text.set_text(t_str)
                
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                time.sleep(0.1)

        else:
            fig, ax, locus_line, start_point, stop_point, table_text = setup_gui("SWEEP")
            
            plt.ion()
            plt.show(block=False)
            
            while plt.fignum_exists(fig.number):
                with state.lock:
                    sr = list(state.sweep_real)
                    si = list(state.sweep_imag)
                    sg = list(state.sweep_gammas)
                
                locus_line.set_data(sr, si)
                if not np.isnan(sr[0]):
                    start_point.set_data([sr[0]], [si[0]])
                if not np.isnan(sr[-1]):
                    stop_point.set_data([sr[-1]], [si[-1]])
                
                table_text.set_text(generate_table_str(state.sweep_freqs, sg))
                
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