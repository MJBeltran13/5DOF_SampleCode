import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import socket
import time
import busio
import RPi.GPIO as GPIO

class IPDisplay:
    def __init__(self):
        # Try different I2C addresses
        self.addresses = [0x3C, 0x3D]  # Common addresses for SSD1306
        self.display = None
        
        # Create the I2C interface
        try:
            print("Initializing I2C...")
            self.i2c = busio.I2C(board.SCL, board.SDA)
            print("I2C initialized successfully")
            
            # Try to find the display
            print("Scanning for I2C devices...")
            while not self.i2c.try_lock():
                pass
            try:
                addresses = self.i2c.scan()
                print(f"Found I2C devices at: {[hex(addr) for addr in addresses]}")
                if not addresses:
                    print("WARNING: No I2C devices found!")
            finally:
                self.i2c.unlock()
            
            # Try each address
            for addr in self.addresses:
                try:
                    print(f"Trying address: 0x{addr:02X}")
                    # Create display with explicit reset pin
                    reset_pin = digitalio.DigitalInOut(board.D4)  # Using GPIO4 as reset pin
                    self.display = adafruit_ssd1306.SSD1306_I2C(
                        128, 128, 
                        self.i2c, 
                        addr=addr,
                        reset=reset_pin
                    )
                    print(f"Display found at address: 0x{addr:02X}")
                    break
                except Exception as e:
                    print(f"Failed at address 0x{addr:02X}: {e}")
                    continue
            
            if self.display is None:
                raise Exception("No display found at any address")
            
            # Clear display and show test pattern
            print("Clearing display...")
            self.display.fill(0)
            self.display.show()
            
            # Draw test pattern
            print("Drawing test pattern...")
            self.display.fill(1)  # Fill with white
            self.display.show()
            time.sleep(1)
            self.display.fill(0)  # Fill with black
            self.display.show()
            
            # Create blank image for drawing
            self.image = Image.new("1", (self.display.width, self.display.height))
            self.draw = ImageDraw.Draw(self.image)
            
            # Load a font
            try:
                self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16)
                self.small_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
                print("Fonts loaded successfully")
            except Exception as e:
                print(f"Warning: Could not load fonts: {e}")
                # Fallback to default font
                self.font = ImageFont.load_default()
                self.small_font = ImageFont.load_default()
            
        except Exception as e:
            print(f"Display initialization error: {e}")
            print("\nTroubleshooting steps:")
            print("1. Check physical connections:")
            print("   - VCC to 3.3V or 5V")
            print("   - GND to Ground")
            print("   - SDA to GPIO2 (SDA)")
            print("   - SCL to GPIO3 (SCL)")
            print("   - RESET to GPIO4 (D4)")
            print("2. Check if I2C is enabled in raspi-config")
            print("3. Try running: sudo i2cdetect -y 1")
            raise

    def get_ip_address(self):
        try:
            # Create a socket object
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Connect to an external server (doesn't actually send any data)
            s.connect(('8.8.8.8', 80))
            # Get the local IP address
            ip_addr = s.getsockname()[0]
            s.close()
            return ip_addr
        except Exception:
            return "No IP found"

    def update_display(self):
        if self.display is None:
            return
            
        try:
            # Clear the image with white background (1 instead of 0)
            self.draw.rectangle((0, 0, self.display.width, self.display.height), outline=1, fill=1)
            
            # Get IP address
            ip = self.get_ip_address()
            
            # Draw the text in black (0 instead of 255)
            self.draw.text((0, 20), "Robot Control", font=self.font, fill=0)
            self.draw.text((0, 45), "IP Address:", font=self.small_font, fill=0)
            self.draw.text((0, 65), ip, font=self.font, fill=0)
            self.draw.text((0, 90), "Port: 5000", font=self.small_font, fill=0)
            
            # Display image
            self.display.image(self.image)
            self.display.show()
        except Exception as e:
            print(f"Display update error: {e}")

    def run(self):
        while True:
            try:
                self.update_display()
                time.sleep(5)  # Update every 5 seconds
            except Exception as e:
                print(f"Display run error: {e}")
                time.sleep(5)  # Wait before retrying

if __name__ == '__main__':
    try:
        display = IPDisplay()
        display.run()
    except Exception as e:
        print(f"Fatal error: {e}") 