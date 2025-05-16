import board
import busio
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1327
import time
import socket

class IPDisplay:
    def __init__(self):
        # I2C setup
        self.i2c = busio.I2C(board.SCL, board.SDA)
        reset_pin = digitalio.DigitalInOut(board.D4)
        
        self.display = adafruit_ssd1327.SSD1327_I2C(
            128, 128, self.i2c, addr=0x3C, reset=reset_pin
        )
        
        self.display.fill(0)
        self.display.show()

        # Image in "L" mode for grayscale
        self.image = Image.new("L", (128, 128))
        self.draw = ImageDraw.Draw(self.image)
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
        self.draw.rectangle((0, 0, 128, 128), fill=0)
        ip = self.get_ip_address()
        self.draw.text((0, 20), "Robot Control", font=self.font, fill=255)
        self.draw.text((0, 50), "IP: " + ip, font=self.font, fill=255)
        self.draw.text((0, 80), "Port: 5000", font=self.font, fill=255)
        self.display.image(self.image)
        self.display.show()

    def run(self):
        while True:
            self.update_display()
            time.sleep(5)

if __name__ == "__main__":
    IPDisplay().run()
