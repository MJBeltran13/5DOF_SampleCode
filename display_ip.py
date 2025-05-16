import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import socket
import time

class IPDisplay:
    def __init__(self):
        # Create the I2C interface
        i2c = board.I2C()
        
        # 128x128 display with hardware I2C
        self.display = adafruit_ssd1306.SSD1306_I2C(128, 128, i2c, addr=0x3C)
        
        # Clear display
        self.display.fill(0)
        self.display.show()
        
        # Create blank image for drawing
        self.image = Image.new("1", (self.display.width, self.display.height))
        self.draw = ImageDraw.Draw(self.image)
        
        # Load a font
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16)
        self.small_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)

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
        # Clear the image
        self.draw.rectangle((0, 0, self.display.width, self.display.height), outline=0, fill=0)
        
        # Get IP address
        ip = self.get_ip_address()
        
        # Draw the text
        self.draw.text((0, 20), "Robot Control", font=self.font, fill=255)
        self.draw.text((0, 45), "IP Address:", font=self.small_font, fill=255)
        self.draw.text((0, 65), ip, font=self.font, fill=255)
        self.draw.text((0, 90), "Port: 5000", font=self.small_font, fill=255)
        
        # Display image
        self.display.image(self.image)
        self.display.show()

    def run(self):
        while True:
            self.update_display()
            time.sleep(5)  # Update every 5 seconds

if __name__ == '__main__':
    display = IPDisplay()
    display.run() 