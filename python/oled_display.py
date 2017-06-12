import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


class OledDisplay:
    def __init__(self, pin=24):
        # 128x32 display with hardware I2C:
        self.dispa = Adafruit_SSD1306.SSD1306_128_32(rst=pin, i2c_address=0x3c)
        self.clear()

    def clear(self):
        self.dispa.begin()
        self.dispa.clear()
        self.dispa.display()

    def display_text(self, text):
        width = self.dispa.width
        height = self.dispa.height
        image = Image.new('1', (width, height))

        # Get drawing object to draw on image.
        draw = ImageDraw.Draw(image)
        padding = -2
        top = padding
        bottom = height - padding

        # Load default font.
        font = ImageFont.load_default()

        # Draw a black filled box to clear the image.
        draw.rectangle((0,0, width, height), outline=0, fill=0)
        for line in text.splitlines():
            # Move left to right keeping track of the current x position for drawing shapes.
            x = 0
            draw.text((x, top), line, font=font, fill=254)
            top+=8

        # Display image.
        self.dispa.image(image)
        self.dispa.display()


if __name__ == '__main__':
    oled = OledDisplay()
    oled.display_text('Line0\nLine1\nLine2')

