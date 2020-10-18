### edubit-audio-spectrogram.py v1.0
### A spectrogram using CLUE microphone with Edu:bit peak meter

### Waterfall FFT demo adapted from
### https://teaandtechtime.com/fft-circuitpython-library/
### to work with ulab on Adafruit CLUE
### https://learn.adafruit.com/ulab-crunch-numbers-fast-with-circuitpython/fft-example-waterfall-spectrum-analyzer  pylint: disable=line-too-long
### with a few extras to use
### 1) Edu:bit NeoPixels as a peak meter for Sound Bit
### 2) Edu:bit RGB LEDs to indicate treble, mids and bass

### Tested with an Adafruit CLUE and CircuitPython 5.3.1

### MIT License

### Copyright (c) 2020 Thomas Schucker
### Copyright (c) 2020 Jeff Epler (ulab conversion)
### Copyright (c) 2020 Kevin J. Walters (edu:bit tweaks)

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


import array

import board
import audiobusio
import displayio
import ulab
import ulab.extras
import ulab.vector
import busio

from cytron_edubit import Edubit
from meters import PeakMeter

### Need to create i2c bus at 100kHz until CircuitPython lowers the default
### https://github.com/adafruit/Adafruit_CircuitPython_CLUE/issues/36
edubit = Edubit(i2c=busio.I2C(board.SCL, board.SDA, frequency=100 * 1000))

peak_meter = PeakMeter(edubit.pixels)

display = board.DISPLAY

# Create a heatmap color palette
PALETTE_LEN = 52
palette = displayio.Palette(PALETTE_LEN)
for idx, color in enumerate((0xff0000, 0xff0a00, 0xff1400, 0xff1e00,
                             0xff2800, 0xff3200, 0xff3c00, 0xff4600,
                             0xff5000, 0xff5a00, 0xff6400, 0xff6e00,
                             0xff7800, 0xff8200, 0xff8c00, 0xff9600,
                             0xffa000, 0xffaa00, 0xffb400, 0xffbe00,
                             0xffc800, 0xffd200, 0xffdc00, 0xffe600,
                             0xfff000, 0xfffa00, 0xfdff00, 0xd7ff00,
                             0xb0ff00, 0x8aff00, 0x65ff00, 0x3eff00,
                             0x17ff00, 0x00ff10, 0x00ff36, 0x00ff5c,
                             0x00ff83, 0x00ffa8, 0x00ffd0, 0x00fff4,
                             0x00a4ff, 0x0094ff, 0x0084ff, 0x0074ff,
                             0x0064ff, 0x0054ff, 0x0044ff, 0x0032ff,
                             0x0022ff, 0x0012ff, 0x0002ff, 0x0000ff)):
    palette[PALETTE_LEN - 1 - idx] = color


class RollingGraph(displayio.TileGrid):
    def __init__(self, scale=2):
        # Create a bitmap with heatmap colors
        self.bitmap = displayio.Bitmap(display.width // scale,
                                       display.height // scale,
                                       len(palette))
        super().__init__(self.bitmap, pixel_shader=palette)

        self.scroll_offset = 0

    def show(self, data):
        y = self.scroll_offset
        bitmap = self.bitmap

        board.DISPLAY.auto_refresh = False
        offset = max(0, (bitmap.width - len(data)) // 2)
        ##offset = 0
        for x in range(min(bitmap.width, len(data))):
            bitmap[x + offset, y] = int(data[x])

        board.DISPLAY.auto_refresh = True

        self.scroll_offset = (y + 1) % self.bitmap.height


pixels_per_bucket = 2
group = displayio.Group(scale=pixels_per_bucket)
graph = RollingGraph(pixels_per_bucket)
fft_size = 256

# Add the TileGrid to the Group
group.append(graph)

# Add the Group to the Display
display.show(group)

# instantiate board mic (CLUE only appears to allow 16000Hz)
mic = audiobusio.PDMIn(board.MICROPHONE_CLOCK, board.MICROPHONE_DATA,
                       sample_rate=16000, bit_depth=16)

#use some extra sample to account for the mic startup
startup_samples = 4
max_vol = 32768
samples_bit = array.array("H", [0] * (fft_size + startup_samples))

### Original code had a more dynamic, tracking approach
min_val = 3
max_val = ulab.vector.log(ulab.array([max_vol * fft_size]))[0] + 0.1

loud_threshold = (max_val - min_val) * 1.4

def loudness(freqs):
    """A crude, very unscientific measure of loudness."""
    return (2.0 * ulab.numerical.mean(freqs)
            + ulab.numerical.max(freqs)
            + ulab.numerical.std(freqs))


# Main Loop
def main():
    while True:
        _ = mic.record(samples_bit, len(samples_bit))
        if startup_samples > 0:
            samples = ulab.array(memoryview(samples_bit)[startup_samples:])
        else:
            samples = ulab.array(samples_bit)

        peak_meter.value = edubit.read_sound_sensor()

        # spectrum() is always nonnegative, but add a tiny value
        # to change any zeros to nonzero numbers (for log())
        full_spectrogram_log = ulab.vector.log(ulab.extras.spectrogram(samples) + 1e-7)
        # Lose the dc component and second half which is mirror image
        spectrogram_log = full_spectrogram_log[1:(fft_size // 2)]

        ### Ignore everything below min_val by setting to zero
        spectrogram_log_floored = spectrogram_log - min_val
        spectrogram_log_floored = (spectrogram_log_floored
                                   * ulab.array(spectrogram_log_floored > 0))

        ### These might be ~63Hz buckets
        bass = spectrogram_log_floored[0:3]
        mid = spectrogram_log_floored[3:12]
        treble_l = spectrogram_log_floored[12:64]
        treble_h = spectrogram_log_floored[64:]

        edubit.set_led(Edubit.GREEN, loudness(bass) > loud_threshold)
        edubit.set_led(Edubit.YELLOW, loudness(mid) > loud_threshold)
        edubit.set_led(Edubit.RED, max(loudness(treble_l), loudness(treble_h)) > loud_threshold)

        # Plot FFT
        data = spectrogram_log_floored * ((PALETTE_LEN - 1) / max_val)
        graph.show(data)

        peak_meter.value = edubit.read_sound_sensor()

main()
