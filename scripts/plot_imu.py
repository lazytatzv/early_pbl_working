#!/usr/bin/env python3
import sys
import os
import re
import matplotlib.pyplot as plt

def parse_log_file(filepath):
    headings = []
    targets = []
    distances = []
    times = []

    # [Status] Dist: 25.4 cm | Heading: 180.2deg (target: 180.0deg)
    # [IMU Test] Status: READY | Current Heading: 180.5 deg | Target: 180.0 deg
    pattern_status = re.compile(r"Dist:\s*([\d\.-]+)\s*cm\s*\|\s*Heading:\s*([\d\.-]+)\s*deg\s*\(target:\s*([\d\.-]+)deg\)")
    pattern_test = re.compile(r"Current Heading:\s*([\d\.-]+)\s*deg\s*\|\s*Target:\s*([\d\.-]+)\s*deg")

    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        for idx, line in enumerate(f):
            # Try parsing status line
            m_status = pattern_status.search(line)
            if m_status:
                try:
                    dist = float(m_status.group(1))
                    heading = float(m_status.group(2))
                    target = float(m_status.group(3))
                    distances.append(dist)
                    headings.append(heading)
                    targets.append(target)
                    times.append(len(times))
                except ValueError:
                    continue
                continue

            # Try parsing test line
            m_test = pattern_test.search(line)
            if m_test:
                try:
                    heading = float(m_test.group(1))
                    target = float(m_test.group(2))
                    headings.append(heading)
                    targets.append(target)
                    distances.append(None)
                    times.append(len(times))
                except ValueError:
                    continue

    return times, headings, targets, distances

def main():
    if len(sys.argv) < 2:
        print("Usage: uv run python scripts/plot_imu.py <log_file_path>")
        print("Alternative (Serial reading): uv run python scripts/plot_imu.py serial [port]")
        return

    arg = sys.argv[1]

    if arg.lower() == "serial":
        # Attempt to read from serial port
        port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM0"
        try:
            import serial
        except ImportError:
            print("pyserial is required for serial mode. Run: uv pip install pyserial")
            return

        print(f"Connecting to {port} at 115200 baud... Press Ctrl+C to stop and plot.")
        headings, targets, distances, times = [], [], [], []
        pattern_status = re.compile(r"Dist:\s*([\d\.-]+)\s*cm\s*\|\s*Heading:\s*([\d\.-]+)\s*deg\s*\(target:\s*([\d\.-]+)deg\)")
        pattern_test = re.compile(r"Current Heading:\s*([\d\.-]+)\s*deg\s*\|\s*Target:\s*([\d\.-]+)\s*deg")

        try:
            ser = serial.Serial(port, 115200, timeout=1.0)
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                print(line)

                m_status = pattern_status.search(line)
                if m_status:
                    try:
                        distances.append(float(m_status.group(1)))
                        headings.append(float(m_status.group(2)))
                        targets.append(float(m_status.group(3)))
                        times.append(len(times))
                    except ValueError:
                        pass
                else:
                    m_test = pattern_test.search(line)
                    if m_test:
                        try:
                            headings.append(float(m_test.group(1)))
                            targets.append(float(m_test.group(2)))
                            distances.append(None)
                            times.append(len(times))
                        except ValueError:
                            pass
        except KeyboardInterrupt:
            print("\nStopped reading. Plotting...")
        except Exception as e:
            print(f"Error reading serial: {e}")
            return
    else:
        # File mode
        if not os.path.exists(arg):
            print(f"File not found: {arg}")
            return
        times, headings, targets, distances = parse_log_file(arg)

    if not headings:
        print("No valid IMU or status data found to plot.")
        return

    import numpy as np

    # Unwrap angles to prevent 0 <-> 360 deg jumps
    headings_rad = np.deg2rad(headings)
    targets_rad = np.deg2rad(targets)
    headings_unwrapped = np.rad2deg(np.unwrap(headings_rad))
    targets_unwrapped = np.rad2deg(np.unwrap(targets_rad))

    # Plot
    fig, ax1 = plt.subplots(figsize=(10, 5))

    color = 'tab:blue'
    ax1.set_xlabel('Data Point Index')
    ax1.set_ylabel('Angle (deg)', color=color)
    ax1.plot(times, headings_unwrapped, label='Current Heading', color='blue', linewidth=1.5)
    ax1.plot(times, targets_unwrapped, label='Target Heading', color='orange', linestyle='--', linewidth=1.5)
    ax1.tick_params(axis='y', labelcolor=color)
    ax1.grid(True)

    # Calculate absolute error (shortest angle distance)
    def angle_diff(a, b):
        diff = (a - b) % 360.0
        if diff > 180.0:
            diff -= 360.0
        return abs(diff)

    errors = [angle_diff(h, t) for h, t in zip(headings, targets)]
    avg_error = sum(errors) / len(errors)
    max_error = max(errors)
    ax1.set_title(f"IMU Heading Accuracy Plot (Unwrapped)\nAvg Error: {avg_error:.2f}° | Max Error: {max_error:.2f}°")

    valid_dists = [(t, d) for t, d in zip(times, distances) if d is not None]
    if valid_dists:
        ax2 = ax1.twinx()
        color = 'tab:red'
        ax2.set_ylabel('Distance (cm)', color=color)
        dist_t, dist_val = zip(*valid_dists)
        ax2.plot(dist_t, dist_val, label='Ultrasonic Distance', color='red', alpha=0.5, linestyle=':')
        ax2.tick_params(axis='y', labelcolor=color)

    fig.tight_layout()
    
    # Save the output plot
    os.makedirs("sim_output", exist_ok=True)
    out_path = "sim_output/imu_accuracy_plot.png"
    plt.savefig(out_path)
    print(f"Plot saved to: {out_path}")
    plt.show()

if __name__ == "__main__":
    main()
