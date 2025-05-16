import board
import busio
import digitalio
import adafruit_ssd1306
import time

# Initialize I2C
print("Initializing I2C...")
i2c = busio.I2C(board.SCL, board.SDA)

# Scan for I2C devices
print("Scanning for I2C devices...")
while not i2c.try_lock():
    pass
try:
    addresses = i2c.scan()
    print(f"Found I2C devices at: {[hex(addr) for addr in addresses]}")
    if not addresses:
        print("WARNING: No I2C devices found!")
finally:
    i2c.unlock()

# Try common OLED display addresses
addresses_to_try = [0x3C, 0x3D]
display = None

for addr in addresses_to_try:
    try:
        print(f"Trying address: 0x{addr:02X}")
        # Try with reset pin
        reset_pin = digitalio.DigitalInOut(board.D4)
        display = adafruit_ssd1306.SSD1306_I2C(
            128, 32,  # Try a common display size (modify if yours is different)
            i2c, 
            addr=addr,
            reset=reset_pin
        )
        print(f"Display found at address: 0x{addr:02X}")
        break
    except Exception as e:
        print(f"Failed at address 0x{addr:02X}: {e}")
        # Try without reset pin
        try:
            print(f"Trying address 0x{addr:02X} without reset pin")
            display = adafruit_ssd1306.SSD1306_I2C(
                128, 32,  # Try a common display size (modify if yours is different)
                i2c, 
                addr=addr
            )
            print(f"Display found at address: 0x{addr:02X} without reset pin")
            break
        except Exception as e2:
            print(f"Failed without reset pin: {e2}")

if display is None:
    print("No display found. Check connections and try again.")
else:
    # Try different display sizes if the first attempt didn't work
    display_sizes = [(128, 32), (128, 64), (128, 128), (96, 16)]
    
    for width, height in display_sizes:
        try:
            print(f"Trying display size: {width}x{height}")
            display = adafruit_ssd1306.SSD1306_I2C(
                width, height,
                i2c,
                addr=display.i2c_device.device_address
            )
            
            # Test pattern
            print("Displaying test pattern...")
            display.fill(0)  # Clear to black
            display.show()
            time.sleep(1)
            
            display.fill(1)  # Fill with white
            display.show()
            time.sleep(1)
            
            display.fill(0)  # Back to black
            
            # Draw a white rectangle border
            for x in range(0, width):
                display.pixel(x, 0, 1)
                display.pixel(x, height-1, 1)
            
            for y in range(0, height):
                display.pixel(0, y, 1)
                display.pixel(width-1, y, 1)
                
            display.show()
            print(f"Success with {width}x{height} display!")
            break
        except Exception as e:
            print(f"Failed with {width}x{height}: {e}")

print("Test complete. Check the display to see if any patterns appeared.") 