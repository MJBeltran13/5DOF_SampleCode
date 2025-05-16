import os
import sys
import time

print("I2C Check Utility for Windows")
print("-----------------------------\n")

# Check for required libraries
print("Checking for required libraries...")
libraries = ["board", "busio", "adafruit_ssd1306", "PIL"]
missing_libs = []

for lib in libraries:
    try:
        __import__(lib)
        print(f"✓ {lib} is installed")
    except ImportError:
        print(f"✗ {lib} is NOT installed")
        missing_libs.append(lib)

if missing_libs:
    print("\nMissing libraries. Please install them with pip:")
    for lib in missing_libs:
        if lib == "PIL":
            print("pip install Pillow")
        elif lib == "adafruit_ssd1306":
            print("pip install adafruit-circuitpython-ssd1306")
        else:
            print(f"pip install {lib.replace('_', '-')}")
    print("\nTip: If you're missing 'board' or 'busio', install adafruit-blinka:")
    print("pip install adafruit-blinka")
    sys.exit(1)

# Try to initialize I2C
print("\nAttempting to initialize I2C...")
import board
import busio

try:
    i2c = busio.I2C(board.SCL, board.SDA)
    print("✓ I2C interface initialized successfully")
    
    # Try to scan for devices
    print("\nScanning for I2C devices...")
    found_devices = False
    
    try:
        while not i2c.try_lock():
            pass
        
        try:
            devices = i2c.scan()
            if devices:
                print(f"✓ Found {len(devices)} I2C device(s):")
                for device in devices:
                    print(f"  - Device at address: 0x{device:02X}")
                found_devices = True
            else:
                print("✗ No I2C devices found on the bus")
        finally:
            i2c.unlock()
    except Exception as e:
        print(f"✗ Error scanning I2C bus: {e}")
    
    # Provide guidance based on results
    if not found_devices:
        print("\nPossible issues:")
        print("1. Display is not connected properly")
        print("2. Display has incorrect power (check voltage)")
        print("3. You might need to install drivers for your I2C adapter")
        print("4. On Windows, you might need special hardware for I2C")
        
except Exception as e:
    print(f"✗ Failed to initialize I2C: {e}")
    print("\nOn Windows, I2C support requires specific hardware:")
    print("- For Raspberry Pi Pico: Connect via USB and use board.GP1/GP0")
    print("- For Arduino: Use an Arduino with CircuitPython firmware")
    print("- For other boards: Check Adafruit Blinka compatibility")

print("\nFor more information, please refer to INSTALL.md") 