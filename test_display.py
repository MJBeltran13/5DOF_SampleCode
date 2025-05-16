import board
import busio
import digitalio
import time
import sys

# More verbose output for debugging
print("Python version:", sys.version)
print("Starting display test with more diagnostics...")

try:
    # Initialize I2C with specific frequency
    print("Initializing I2C...")
    i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)  # Lower frequency for stability
    print("I2C initialized successfully")
    
    # Scan for I2C devices
    print("Scanning for I2C devices...")
    devices_found = False
    
    # Try multiple times to scan for devices (sometimes first scan fails)
    for attempt in range(3):
        print(f"Scan attempt {attempt+1}...")
        while not i2c.try_lock():
            pass
        try:
            addresses = i2c.scan()
            if addresses:
                devices_found = True
                print(f"Found I2C devices at: {[hex(addr) for addr in addresses]}")
                break
            else:
                print("No devices found on attempt", attempt+1)
        finally:
            i2c.unlock()
        time.sleep(1)  # Wait before next attempt
    
    if not devices_found:
        print("WARNING: No I2C devices found after multiple attempts!")
        print("\nCheck your connections:")
        print("1. Make sure the display is properly powered (check VCC and GND)")
        print("2. Verify SDA and SCL connections")
        print("3. Try with pull-up resistors if not already present")
        sys.exit(1)
    
    # Import display libraries only after confirming I2C works
    print("Importing display libraries...")
    import adafruit_ssd1306
    from PIL import Image, ImageDraw, ImageFont
    print("Display libraries imported successfully")
    
    # Try to initialize display with common configurations
    print("\n=== Testing different display configurations ===")
    
    # Common OLED display addresses and sizes
    configs = [
        {"addr": 0x3C, "width": 128, "height": 32, "reset": True},
        {"addr": 0x3C, "width": 128, "height": 64, "reset": True},
        {"addr": 0x3C, "width": 128, "height": 128, "reset": True},
        {"addr": 0x3D, "width": 128, "height": 32, "reset": True},
        {"addr": 0x3D, "width": 128, "height": 64, "reset": True},
        {"addr": 0x3D, "width": 128, "height": 128, "reset": True},
        # Try without reset pin
        {"addr": 0x3C, "width": 128, "height": 32, "reset": False},
        {"addr": 0x3C, "width": 128, "height": 64, "reset": False},
        {"addr": 0x3D, "width": 128, "height": 32, "reset": False},
        {"addr": 0x3D, "width": 128, "height": 64, "reset": False},
    ]
    
    success = False
    
    for config in configs:
        try:
            print(f"\nTrying: Address=0x{config['addr']:02X}, Size={config['width']}x{config['height']}, Reset={config['reset']}")
            
            if config['reset']:
                # Use reset pin
                reset_pin = digitalio.DigitalInOut(board.D4)
                print("Reset pin initialized")
                
                # Explicit reset sequence
                reset_pin.switch_to_output(value=True)
                time.sleep(0.1)
                reset_pin.value = False
                time.sleep(0.1)
                reset_pin.value = True
                time.sleep(0.1)
                
                display = adafruit_ssd1306.SSD1306_I2C(
                    config['width'], config['height'],
                    i2c, 
                    addr=config['addr'],
                    reset=reset_pin
                )
            else:
                # Try without reset pin
                display = adafruit_ssd1306.SSD1306_I2C(
                    config['width'], config['height'],
                    i2c, 
                    addr=config['addr']
                )
            
            print("Display initialized successfully!")
            
            # Explicit display init sequence
            display.poweron()
            time.sleep(0.1)
            display.fill(0)
            display.show()
            time.sleep(0.5)
            
            # Draw a test pattern
            print("Drawing test pattern...")
            display.contrast(255)  # Max contrast
            
            # Test 1: Full white screen
            display.fill(1)
            display.show()
            print("Test 1: Display should show all white")
            time.sleep(2)
            
            # Test 2: Full black screen
            display.fill(0)
            display.show()
            print("Test 2: Display should show all black")
            time.sleep(2)
            
            # Test 3: Checkerboard pattern
            for y in range(0, config['height'], 8):
                for x in range(0, config['width'], 8):
                    if (x // 8 + y // 8) % 2 == 0:
                        for dy in range(8):
                            for dx in range(8):
                                if y+dy < config['height'] and x+dx < config['width']:
                                    display.pixel(x+dx, y+dy, 1)
            display.show()
            print("Test 3: Display should show checkerboard pattern")
            time.sleep(2)
            
            # Test 4: Border
            display.fill(0)
            for x in range(config['width']):
                display.pixel(x, 0, 1)
                display.pixel(x, config['height']-1, 1)
            for y in range(config['height']):
                display.pixel(0, y, 1)
                display.pixel(config['width']-1, y, 1)
            display.show()
            print("Test 4: Display should show border")
            
            print(f"SUCCESS with config: Address=0x{config['addr']:02X}, Size={config['width']}x{config['height']}, Reset={config['reset']}")
            success = True
            break
            
        except Exception as e:
            print(f"Failed with configuration: {e}")
            continue
    
    if not success:
        print("\n===== No display configuration worked =====")
        print("Try these troubleshooting steps:")
        print("1. Check power supply - make sure display has stable power")
        print("2. Check all connections carefully")
        print("3. Try with a different display if available")
        print("4. Check if the display is damaged or defective")
        print("5. Make sure your board has I2C capability properly configured")
    else:
        print("\n===== Display test completed successfully =====")
        print(f"Working configuration: Address=0x{config['addr']:02X}, Size={config['width']}x{config['height']}, Reset={config['reset']}")
        print("Use these values in your main program")

except Exception as e:
    print(f"Error during test: {e}")
    import traceback
    traceback.print_exc()
    print("\nSomething went wrong with the I2C communication or display initialization.")
    print("Check hardware connections and try again.") 