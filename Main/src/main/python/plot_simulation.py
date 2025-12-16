import pandas as pd
import matplotlib.pyplot as plt

def plot_simulation():
    try:
        df = pd.read_csv('simulation_results.csv')
    except FileNotFoundError:
        print("Error: simulation_results.csv not found.")
        return

    plt.figure(figsize=(10, 8))
    
    # Plot 1: Position Tracking
    plt.subplot(3, 1, 1)
    plt.plot(df['t'], df['ref_pos'], 'k--', label='Reference', linewidth=1.5)
    plt.plot(df['t'], df['true_pos'], 'b-', label='True State', linewidth=1.5, alpha=0.8)
    plt.plot(df['t'], df['est_pos'], 'r:', label='KF Estimate', linewidth=1.5)
    plt.title('Position Tracking (S-Curve Profile)')
    plt.ylabel('Position (m)')
    plt.legend()
    plt.grid(True)

    # Plot 2: Velocity Tracking
    plt.subplot(3, 1, 2)
    plt.plot(df['t'], df['ref_vel'], 'k--', label='Reference')
    plt.plot(df['t'], df['true_vel'], 'b-', label='True Velocity', alpha=0.8)
    plt.plot(df['t'], df['est_vel'], 'r:', label='KF Estimate')
    plt.title('Velocity Tracking')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.grid(True)

    # Plot 3: Control Input
    plt.subplot(3, 1, 3)
    plt.plot(df['t'], df['u_volts'], 'g-', label='Voltage')
    plt.axhline(y=12, color='r', linestyle='--', alpha=0.3)
    plt.axhline(y=-12, color='r', linestyle='--', alpha=0.3)
    plt.title('Control Effort (Feedforward + LQR)')
    plt.ylabel('Voltage (V)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('simulation_result.png')
    print("Plot saved to simulation_result.png")

if __name__ == "__main__":
    plot_simulation()
