### edubit-lidar-servo-distance.py v1.0
### A lidar using an Edu:bit controlled servo with tinyLiDAR ToF distance sensor

### Tested with an Adafruit CLUE and CircuitPython 5.3.1

### MIT License

### Copyright (c) 2020 Kevin J. Walters

### Permission is hereby granted, free of charge, to any person obtaining a copy
### of this software and associated documentation files (the "Software"), to deal
### in the Software without restriction, including without limitation the rights
### to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
### copies of the Software, and to permit persons to whom the Software is
### furnished to do so, subject to the following conditions:

### The above copyright notice and this permission notice shall be included in all
### copies or substantial portions of the Software.

### THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
### IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
### FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
### AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
### LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
### OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
### SOFTWARE.


import time
import math

import busio
import board
import displayio

from adafruit_bus_device.i2c_device import I2CDevice

from cytron_edubit import Edubit
from meters import PeakMeter


TINYLIDAR_ADDRESS = 0x10
TL_SETCM = b"MC"
TL_GETDISTANCE = b"D"


def continuousMode(tl_dev):
    with tl_dev:
        tl_dev.write(TL_SETCM)


def readDistance(tl_dev):
    """Return distance in mm from tinyLiDAR."""
    getdistancebuf = bytearray([255] * 2)
    with tl_dev:
        tl_dev.write(TL_GETDISTANCE)
        tl_dev.readinto(getdistancebuf)
    return (getdistancebuf[0] << 8) + getdistancebuf[1]


def polarToCart(angle_deg, dist):
    cart_x = round(math.cos(math.radians(angle_deg)) * dist)
    cart_y = round(math.sin(math.radians(angle_deg)) * dist)
    return (cart_x, cart_y)


### Need to create i2c bus at 100kHz until CircuitPython lowers the default
### https://github.com/adafruit/Adafruit_CircuitPython_CLUE/issues/36
i2c=busio.I2C(board.SCL, board.SDA, frequency=100 * 1000)
edubit = Edubit(i2c=i2c)
peak_meter = PeakMeter(edubit.pixels)

LIDAR_SERVO = Edubit.S3
lidar_device = I2CDevice(i2c, TINYLIDAR_ADDRESS)

display = board.DISPLAY

PIXEL_SIZE = 2

### Lidar background
palette_bg = displayio.Palette(2)
palette_bg[0] = 0x000000
palette_bg[1] = 0xc0c0c0
screen_bg = displayio.Bitmap(display.width,
                             display.height,
                             len(palette_bg))
tg_screen_bg = displayio.TileGrid(screen_bg, pixel_shader=palette_bg)

### Lidar screen for return data
TARGET_COLOURS = (0x000000, 0x102010, 0x204020, 0x306030,
                  0x408040, 0x60c060, 0x70e070, 0x80ff80)
palette = displayio.Palette(len(TARGET_COLOURS))
for p_idx, color in enumerate(TARGET_COLOURS):
    palette[p_idx] = color
palette.make_transparent(0)
screen = displayio.Bitmap(display.width // PIXEL_SIZE,
                          display.height // PIXEL_SIZE,
                          len(palette))
tg_screen = displayio.TileGrid(screen, pixel_shader=palette)
screen_group = displayio.Group(scale=PIXEL_SIZE, max_size=1)
screen_group.append(tg_screen)

main_group = displayio.Group(max_size=2)
main_group.append(tg_screen_bg)
main_group.append(screen_group)

display.show(main_group)


def drawBackground(bmp,
                   origin_x, origin_y,
                   min_angle, max_angle,
                   max_range,
                   *,
                   orientation=1,
                   arcs=4,
                   col_idx=1):
    """Create a background for the lidar screen with a few simple white arcs."""
    ### pylint: disable=too-many-locals

    angle_range = max_angle - min_angle
    for arc in range(1, arcs + 1):
        ratio = arc / arcs
        steps = round(ratio * max_range / 5)
        dist = ratio * max_range
        for ang_step in range(steps + 1):
            ang = min_angle + angle_range * ang_step / steps
            dd_x, dd_y = polarToCart(ang, dist)
            pp_x = orientation * round(dd_x)
            pp_y = orientation * -round(dd_y)
            bmp[origin_x + pp_x, origin_y + pp_y] = col_idx


def updateScreen(idx, new_x, new_y):
    """Redraw all pixels along this radial with decaying brightness."""

    if screen_x[idx] is not None and screen_y[idx] is not None:
        pixels = { (screen_x[idx], screen_y[idx]):0}
    else:
        pixels = {}

    screen_x[idx] = new_x
    screen_y[idx] = new_y

    ### Iterate over previous scanned values
    ### Aiming for total brightness per pixel of approximately 0 - 255
    new_brightness = 160
    for sw_idx in range(persist_sweeps):
        pp_x = screen_x[idx - sw_idx * values]
        pp_y = screen_y[idx - sw_idx * values]
        if pp_x is not None and pp_y is not None:
            try:
                pixels[(pp_x, pp_y)] += new_brightness
            except KeyError:
                pixels[(pp_x, pp_y)] = new_brightness

        new_brightness /= 1.3

    ### Repaint all the pixels with new brightness values
    for (pp_x, pp_y), bri in pixels.items():
        screen[lidar_origin_x + pp_x,
               lidar_origin_y + pp_y] = min(round(bri), 255) >> 5

servo_offset = 11  ### Value to add for tinyLiDAR to be perpendicular at centre

centre = 90
sweep_min = centre - 35
sweep_max = centre + 35
values = sweep_max - sweep_min + 1
angle_step = 2
SERVO_MOVE_TIME = 0.025

persist_sweeps = 4
distances = [None] * (sweep_max + 1)
### Ring buffer for distance values
screen_x = [None] * values * persist_sweeps
screen_y = [None] * values * persist_sweeps

dist_threshold_mm = 4  ### ignore distance values at or below this
range_mm = 150.0
max_pix_radius = 190.0 / PIXEL_SIZE
dist_pix_factor = range_mm / 190.0 / PIXEL_SIZE

lidar_origin_x = display.width // 2 // PIXEL_SIZE
lidar_origin_y = 0

lidar_orientation = -1  ### -1 is upside_down

drawBackground(screen_bg,
               lidar_origin_x * PIXEL_SIZE, lidar_origin_y,
               sweep_min, sweep_max,
               max_pix_radius * PIXEL_SIZE,  ### background using actual pixels
               orientation=lidar_orientation)


### TiNYLidar continuous mode
continuousMode(lidar_device)

### Set servo to centre position with offset to allow the direction
### to be manually verified
edubit.set_servo_position(LIDAR_SERVO, centre + servo_offset)
time.sleep(3)


### Sweep in alternate directions with angles spaced so the reverse
### sweep interleaves the forward one, plotting all values as they are
### received
sweep_idx = 0
while True:
    ### Calculate starting offset into x/y ring buffers
    s_d_offset = sweep_idx % persist_sweeps * values

    if sweep_idx % 2 == 0:  ### Forward
        sweep_range = range(sweep_min, sweep_max + 1, angle_step)
    else:  ### Reverse
        sweep_range = range(sweep_max - angle_step // 2, sweep_min, -angle_step)

    for angle in sweep_range:
        edubit.set_servo_position(LIDAR_SERVO, angle + servo_offset)
        time.sleep(SERVO_MOVE_TIME)
        dist_mm = readDistance(lidar_device)
        distances[angle] = dist_mm
        p_x = p_y = None
        if dist_mm is not None and dist_mm > dist_threshold_mm:
            pix_dist = dist_mm * dist_pix_factor
            if pix_dist <= max_pix_radius:
                d_x, d_y = polarToCart(angle, pix_dist)
                p_x = lidar_orientation * round(d_x)
                p_y = lidar_orientation * -round(d_y)

        s_idx = angle - sweep_min + s_d_offset
        updateScreen(s_idx, p_x, p_y)

    sweep_idx += 1
