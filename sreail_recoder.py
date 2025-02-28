import serial
import time

def main():
    port = '/dev/ttyACM0'
    baudrate = 115200
    timeout = 1  # seconds

    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"Connected to {port} at {baudrate} baud.")
    except Exception as e:
        print(f"Failed to open {port}: {e}")
        return

    # Allow some time for the connection to be established
    time.sleep(2)

    output_file = "serial_data.txt"
    try:
        with open(output_file, "a") as file:
            print(f"Logging serial data to {output_file}. Press Ctrl+C to stop.")
            while True:
                if ser.in_waiting:
                    data = ser.readline().decode("utf-8", errors="replace").strip()
                    if data:
                        print(data)
                        file.write(data + "\n")
                else:
                    time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting and closing the file.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ser.close()

if __name__ == "__main__":
    main()