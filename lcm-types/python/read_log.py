import lcm
from actuator_response_t import actuator_response_t
import matplotlib.pyplot as plt

# Initialize a dictionary to store data for each motor ID
motor_data = {motor_id: {"timestamps": [], "positions": [], "velocities": [], "currents": [], "time_diffs": []} for motor_id in range(1, 13)}

def main():
    # Specify the LCM log file
    log_file = "/home/lenovo/projects/IUST_SOFTWARE/software_16oct/debug_tools/lcmlog-2024-12-14.02"

    # Open the LCM log
    try:
        log = lcm.EventLog(log_file, "r")
    except FileNotFoundError:
        print(f"Log file {log_file} not found!")
        return

    # Read the log
    for event in log:
        if event.channel == "RESP":  # Replace with the correct LCM channel
            msg = actuator_response_t.decode(event.data)
            
            # Check if motor ID is within expected range
            if msg.id in motor_data:
                current_time = event.timestamp / 1e6  # Convert to milliseconds
                timestamps = motor_data[msg.id]["timestamps"]

                # Calculate time difference if there's a previous timestamp
                if timestamps:
                    time_diff = current_time - timestamps[-1]
                    motor_data[msg.id]["time_diffs"].append(time_diff)
                else:
                    motor_data[msg.id]["time_diffs"].append(0)  # First entry has no difference

                # Append data
                timestamps.append(current_time)
                motor_data[msg.id]["positions"].append(msg.position)
                motor_data[msg.id]["velocities"].append(msg.velocity)
                motor_data[msg.id]["currents"].append(msg.current)

    # Plot data for each motor
    for motor_id, data in motor_data.items():
        if data["timestamps"]:  # Check if there is data for the motor
            plt.figure(figsize=(12, 12))

            # Plot position
            plt.subplot(4, 1, 1)
            plt.plot(data["timestamps"], data["positions"], label=f"Motor {motor_id} Position", color="blue")
            plt.xlabel("Time (ms)")
            plt.ylabel("Position")
            plt.title(f"Motor {motor_id} - Position")
            plt.grid(True)
            plt.legend()

            # Plot velocity
            plt.subplot(4, 1, 2)
            plt.plot(data["timestamps"], data["velocities"], label=f"Motor {motor_id} Velocity", color="green")
            plt.xlabel("Time (ms)")
            plt.ylabel("Velocity")
            plt.title(f"Motor {motor_id} - Velocity")
            plt.grid(True)
            plt.legend()

            # Plot current
            plt.subplot(4, 1, 3)
            plt.plot(data["timestamps"], data["currents"], label=f"Motor {motor_id} Current", color="red")
            plt.xlabel("Time (ms)")
            plt.ylabel("Current")
            plt.title(f"Motor {motor_id} - Current")
            plt.grid(True)
            plt.legend()

            # Plot time differences
            plt.subplot(4, 1, 4)
            plt.plot(data["timestamps"][1:], data["time_diffs"][1:], label=f"Motor {motor_id} Time Steps", color="purple")
            plt.xlabel("Time (ms)")
            plt.ylabel("Time Step Difference (ms)")
            plt.title(f"Motor {motor_id} - Time Step Differences")
            plt.grid(True)
            plt.legend()

            plt.tight_layout()
            plt.show()
        else:
            print(f"No data found for Motor {motor_id}")

if __name__ == "__main__":
    main()
