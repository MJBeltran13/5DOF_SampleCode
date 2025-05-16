# Display Setup Instructions

This guide explains how to set up the OLED display for the robot control interface.

## Hardware Requirements

- Raspberry Pi or compatible board
- 128x64 OLED display (SSD1306 or similar controller)
- Jumper wires

## Wiring

Connect the OLED display to your board:

| Display Pin | Board Pin   | Required |
|-------------|-------------|----------|
| VCC         | 3.3V or 5V  | Yes      |
| GND         | GND         | Yes      |
| SCL         | GPIO3 (SCL) | Yes      |
| SDA         | GPIO2 (SDA) | Yes      |
| RST         | GPIO4       | No       |

**Note**: The RST (reset) pin is optional. The current code does not use a reset pin.

## Software Installation

1. Install required Python libraries:

```bash
pip install adafruit-blinka
pip install adafruit-circuitpython-ssd1306
pip install Pillow
```

2. Enable I2C interface (on Raspberry Pi):

```bash
sudo raspi-config
```

Navigate to: Interface Options -> I2C -> Enable

3. Verify the display is detected:

```bash
sudo i2cdetect -y 1
```

You should see a device at address 0x3C or 0x3D.

## Troubleshooting

### No Module Named 'board'

If you get this error, you need to install the adafruit-blinka library:

```bash
pip install adafruit-blinka
```

### Display Not Showing Anything

1. Check your wiring connections
2. Verify I2C is enabled
3. Run the test script to try different configurations:

```bash
python test_display.py
```

### Display Size Issues

If your display is not 128x64, modify the display_ip.py file:

```python
# Change these values to match your display
WIDTH = 128
HEIGHT = 64  # Change this to 32 or 128 if needed
```

## Updating the Code

If you have a different display size or model, modify the display initialization in display_ip.py:

```python
self.display = adafruit_ssd1306.SSD1306_I2C(
    WIDTH, HEIGHT, self.i2c, addr=0x3C  # No reset pin needed
)
```

If your display uses a different I2C address (usually 0x3D), change it here:

```python
self.display = adafruit_ssd1306.SSD1306_I2C(
    WIDTH, HEIGHT, self.i2c, addr=0x3D  # Try 0x3D if 0x3C doesn't work
)
```

If you do need to use a reset pin, add it back like this:

```python
reset_pin = digitalio.DigitalInOut(board.D4)
self.display = adafruit_ssd1306.SSD1306_I2C(
    WIDTH, HEIGHT, self.i2c, addr=0x3C, reset=reset_pin
)
```

## Additional Resources

- [Adafruit SSD1306 Library Documentation](https://docs.circuitpython.org/projects/ssd1306/en/latest/)
- [Raspberry Pi I2C Documentation](https://www.raspberrypi.org/documentation/hardware/raspberrypi/i2c/README.md) 