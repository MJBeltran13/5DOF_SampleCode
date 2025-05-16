import board
import busio
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import socket
import time

# Display configuration
WIDTH = 128
HEIGHT = 64  # Standard OLED display size

class IPDisplay:
    def __init__(self):
        # I2C setup
        self.i2c = busio.I2C(board.SCL, board.SDA)
        
        # Create display without reset pin
        self.display = adafruit_ssd1306.SSD1306_I2C(
            WIDTH, HEIGHT, self.i2c, addr=0x3C
        )
        
        # Clear the display
        self.display.fill(0)
        self.display.show()

        # Create image for drawing (mode '1' is 1-bit color)
        self.image = Image.new("1", (WIDTH, HEIGHT))
        self.draw = ImageDraw.Draw(self.image)
        
        # Load default font
        self.font = ImageFont.load_default()

    def get_ip_address(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "No IP"

    def update_display(self):
        # Clear display (fill with white for better visibility)
        self.draw.rectangle((0, 0, WIDTH, HEIGHT), outline=1, fill=1)
        
        # Get IP address
        ip = self.get_ip_address()
        
        # Draw text in black on white background - adjusted for smaller height
        self.draw.text((0, 5), "Robot Control", font=self.font, fill=0)
        self.draw.text((0, 25), "IP: " + ip, font=self.font, fill=0)
        self.draw.text((0, 45), "Port: 5000", font=self.font, fill=0)
        
        # Display the image
        self.display.image(self.image)
        self.display.show()

    def run(self):
        while True:
            self.update_display()
            time.sleep(5)

if __name__ == "__main__":
    IPDisplay().run()
