Introduction
============

This is a `CircuitPython <https://circuitpython.org/>`_ library for the
`Cytron Edu:bit <https://www.cytron.io/p-edubit-training-and-project-kit-for-microbit>`_
based on https://github.com/Bhavithiran97/micropython-edubit .
The Edu:bit uses i2c address `0x08`.

Dependencies
=============

This library depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
* `Register <https://github.com/adafruit/Adafruit_CircuitPython_Register>`_
* `Adafruit CircuitPython NeoPixel <https://github.com/adafruit/Adafruit_CircuitPython_NeoPixel>`_


Usage Example
=============

.. code-block:: python

    import board
    import busio
    from cytron_edubit import Edubit

    ##i2c = board.I2C()  # CP 5.3.1 has this running at 400kHz
    i2c = busio.I2C(board.SCL, board.SDA, frequency=100 * 1000)
    edubit = Edubit(i2c=i2c)
    edubit.pixels.fill((32 ,0, 32))  # Set RGB pixels to magenta
    print("Board voltage is", edubit.read_Vin(), "volts")
    while True:
        sound_level = edubit.read_sound_sensor()
        potentiometer = edubit.read_pot_value()
        ir_triggered = edubit.read_IR_sensor()
        edubit.set_led(Edubit.RED, sound_level > 15000)
        edubit.set_led(Edubit.YELLOW, potentiometer > 32768)
        edubit.set_led(Edubit.GREEN, ir_triggered)

