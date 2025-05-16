import board
import busio
import time

def check_i2c():
    print("Checking I2C connections...")
    try:
        # Initialize I2C
        i2c = busio.I2C(board.SCL, board.SDA)
        
        # Scan for devices
        print("Scanning I2C bus...")
        while not i2c.try_lock():
            pass
        
        try:
            addresses = i2c.scan()
            print(f"Found {len(addresses)} I2C device(s):")
            for address in addresses:
                print(f"Device found at address: 0x{address:02X}")
        finally:
            i2c.unlock()
            
    except Exception as e:
        print(f"Error: {e}")
        print("\nTroubleshooting steps:")
        print("1. Check if I2C is enabled:")
        print("   Run: sudo raspi-config")
        print("   Navigate to: Interface Options -> I2C -> Enable")
        print("\n2. Check physical connections:")
        print("   - VCC to 3.3V or 5V")
        print("   - GND to Ground")
        print("   - SDA to GPIO2 (SDA)")
        print("   - SCL to GPIO3 (SCL)")
        print("\n3. Check if display is powered:")
        print("   - LED should be on if display has one")
        print("   - Check voltage with multimeter if possible")
        print("\n4. Try different I2C address:")
        print("   Common addresses are: 0x3C or 0x3D")

if __name__ == "__main__":
    check_i2c() 