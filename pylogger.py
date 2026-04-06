import serial
import struct
import csv
import time

# --- Configuration ---
SERIAL_PORT = 'COM8'  # Change to your ESP32 port
BAUD_RATE = 115200    # Matches your latest Arduino setting
CSV_FILE = "hil_test_results.csv"
PACKET_SIZE = 7
HEADER = 0xBB

def run_logger():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE}...")
        
        # Initialize start_time as None
        start_time = None

        with open(CSV_FILE, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "PWM", "Signal_995", "Distance_mm"])
            
            print("Waiting for motor movement/data...")

            while True:
                byte_in = ser.read(1)
                if not byte_in:
                    continue 

                if byte_in[0] == HEADER:
                    payload = ser.read(PACKET_SIZE - 1)
                    
                    if len(payload) == 6:
                        try:
                            pwm, signal, distance = struct.unpack('<BBf', payload)
                            
                            # Set start_time only when the FIRST valid packet arrives
                            if start_time is None:
                                start_time = time.time()
                                print("--- Recording Started ---")

                            elapsed = time.time() - start_time
                            writer.writerow([round(elapsed, 4), pwm, signal, round(distance, 3)])
                            
                            # Print status every ~0.5 seconds
                            if int(elapsed * 10) % 5 == 0:
                                print(f"Time: {elapsed:.2f}s | Dist: {distance:.2f}mm | PWM: {pwm}")
                                
                        except struct.error:
                            continue

    except KeyboardInterrupt:
        print("\nLogging stopped by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    run_logger()