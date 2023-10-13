# Display control module for Alpha Industry
# Author: Anthony Nguyen
import os
import board
import time
import terminalio
import displayio
import busio
import adafruit_st7789
from adafruit_display_shapes.rect import Rect
from adafruit_display_shapes.circle import Circle
from adafruit_display_shapes.roundrect import RoundRect
from adafruit_display_shapes.triangle import Triangle
from adafruit_display_shapes.sparkline import Sparkline
from adafruit_display_shapes.line import Line
from adafruit_display_text import label
import digitalio

class Display:
    def __init__(self)-> None:
        displayio.release_displays()
        #Initialize ports connections and SPI bus
        tft_cs = board.IO44
        tft_dc = board.IO43
        tft_res = board.IO3
        tft_blk = board.IO37
        spi_mosi = board.IO35
        spi_clk = board.IO36

        pin_blk = digitalio.DigitalInOut(tft_blk)
        pin_blk.direction = digitalio.Direction.OUTPUT
        pin_blk.value = True

        spi = busio.SPI(spi_clk, MOSI=spi_mosi)

        display_bus = displayio.FourWire(spi, command=tft_dc, chip_select=tft_cs, reset=tft_res)
        self.display = adafruit_st7789.ST7789(display_bus, width=320, height=240,rotation=270,)
        self.number = 0

        # splash is instance used to add and remove image on screen
        self.splash = displayio.Group(scale = 2)
        self.display.show(self.splash)
        color_bitmap = displayio.Bitmap(self.display.width, self.display.height, 1)
        color_palette = displayio.Palette(1)
        color_palette[0] = 0x000000 # set the background to all black
        self.bg_sprite = displayio.TileGrid(color_bitmap, pixel_shader=color_palette, x=0, y=0)
        self.splash.append(self.bg_sprite)

    def generate_triangle(self,a,b,c,d,e,f,fill = 0x000000, outline=0xFFFFFF,token = False):
        x = fill
        y = outline
        triangle = Triangle(a, b, c, d, e, f, fill =x , outline=y)
        self.splash.append(triangle)
        self.number = self.number +1
        if token == True:
            return self.splash.index(triangle)

    def generate_circle(self,  centerX, centerY, radius, fill=0x000000, outline=0xFF0000,token = False):
        x = fill
        y = outline
        circle = Circle(centerX, centerY, radius, fill=x  , outline=y)
        self.splash.append(circle)
        self.number = self.number +1
        if token == True:
            return self.splash.index(circle)

    def generate_line(self, a,b,c,d, color = 0xFFFFFF,token = False):
        x = color
        line = Line(a,b,c,d,color = x)
        self.splash.append(line)
        self.number = self.number +1
        if token == True:
            return self.splash.index(line)

    def generate_text(self,x,y, text,token = False):
        text_area = label.Label(terminalio.FONT, text=text)
        text_area.x = x
        text_area.y = y
        self.splash.append(text_area)
        self.number = self.number +1
        if token == True:
            return self.splash.index(text_area)

    def pop(self,index):
        self.splash.pop(index)
        self.number = self.number -1

    def refresh(self):
        #for i in range(self.number):
        #    if self.number-i != 0:
        #        self.splash.pop(self.number-i)
        while self.number != 0:
            self.pop(self.number)
        #self.number = 0






