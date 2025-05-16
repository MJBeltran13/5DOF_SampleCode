import board
import busio
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import socket
import time

class IPDisplay:
    def __init__(self):
        # I2C setup
        self.i2c = busio.I2C(board.SCL, board.SDA)
        reset_pin = digitalio.DigitalInOut(board.D4)
        
        # Create display with SSD1306 driver (128x128 pixels)
        self.display = adafruit_ssd1306.SSD1306_I2C(
            128, 128, self.i2c, addr=0x3C, reset=reset_pin
        )
        
        # Clear the display
        self.display.fill(0)
        self.display.show()

        # Create image for drawing (mode '1' is 1-bit color)
        self.image = Image.new("1", (128, 128))
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
        self.draw.rectangle((0, 0, 128, 128), outline=1, fill=1)
        
        # Get IP address
        ip = self.get_ip_address()
        
        # Draw text in black on white background
        self.draw.text((0, 20), "Robot Control", font=self.font, fill=0)
        self.draw.text((0, 50), "IP: " + ip, font=self.font, fill=0)
        self.draw.text((0, 80), "Port: 5000", font=self.font, fill=0)
        
        # Display the image
        self.display.image(self.image)
        self.display.show()

    def run(self):
        while True:
            self.update_display()
            time.sleep(5)

if __name__ == "__main__":
    IPDisplay().run()
