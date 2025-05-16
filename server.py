from robot_control import app
import socket
import serial
import time

def get_ip_address():
    """Get the local IP address"""
    try:
        # Create a socket to determine the IP address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))  # Connect to Google DNS
        ip = s.getsockname()[0]     # Get local IP
        s.close()
        return ip
    except Exception as e:
        print(f"Error getting IP: {e}")
        return '127.0.0.1'  # Return localhost if failed

def send_ip_to_esp32(port='COM11', baudrate=115200):
    """Send IP address to ESP32"""
    try:
        # Configure serial connection with same settings as robot_control.py
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        # Wait for ESP32 to initialize (same as robot_control.py)
        time.sleep(2)
        
        # Get IP address
        ip = get_ip_address()
        print(f"Local IP address: {ip}")
        
        # Try sending IP up to 3 times
        max_retries = 3
        for attempt in range(max_retries):
            print(f"Attempt {attempt + 1} to send IP...")
            
            # Clear any pending data
            ser.reset_input_buffer()
            
            # Send IP to ESP32
            command = f"IP:{ip}\n"
            ser.write(command.encode())
            
            # Wait for acknowledgment
            response = ser.readline().decode().strip()
            print(f"Response: {response}")
            
            if "Received server IP" in response:
                ack = ser.readline().decode().strip()  # Read the "OK"
                if ack == "OK":
                    print("ESP32 acknowledged IP address")
                    break
            else:
                if attempt < max_retries - 1:  # If not last attempt
                    print("No acknowledgment, retrying...")
                    time.sleep(1)  # Wait before retry
                else:
                    print("Failed to get acknowledgment from ESP32")
        
        # Close the connection
        ser.close()
        
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        print(f"Make sure ESP32 is connected to {port}")
        print("Available ports:")
        ports = serial.tools.list_ports.comports()
        for p in ports:
            print(f"  {p.device}: {p.description}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    # Send IP to ESP32 with COM11 at 115200 baud
    send_ip_to_esp32(port='COM11', baudrate=115200)
    
    # Run the Flask server
    print("Starting server on port 5000...")
    app.run(host='0.0.0.0', port=5000, debug=True)