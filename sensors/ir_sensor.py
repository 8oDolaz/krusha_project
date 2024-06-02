import numpy as np
from scipy.optimize import curve_fit

import Adafruit_MCP3008


class MCP:
    '''
    Analogue-to-digital convertor used to read data from Sharp IR sensor.
    This class should be used for IR implementation.

    For more indepth details look at
    https://storage.googleapis.com/media.amperka.com/products/chip-mcp3008/media/mcp3008-datasheet.pdf
    '''
    def __init__(self, clk, cs, miso, mosi, channels=8):
        self.mcp = Adafruit_MCP3008.MCP3008(
            clk=clk,
            cs=cs,
            miso=miso,
            mosi=mosi,
        )

        self.channels = channels


class Sharp_IR:
    def __init__(self, x, y, theta, mcp, mcp_channel,
                 min=4, max=30, scale=2.5, const=1, coeff=28, power=1):
        '''
        x, y, theta : int, int, int
            position relative to robot (?)

        mcp : MCP (implemented above)
            ADC to read data

        mcp_channel : int
            channel to ADC to read data from

        min, max : int, int
            mininal and maximal threshold of mesuarements

        scale, const, coeff, power : float, float, float, int
            coefficients for output curve of sharp IR
        '''
        self.x, self.y, self.theta = x, y, theta

        self.mcp = mcp
        if not 0 < mcp_channel < mcp.channels - 1:
            raise ValueError('Invalid mcp channel provided. Option are: 0..7')
        self.channel = mcp_channel

        self.min, self.max = min, max
        self.scale = scale
        self.coeff = coeff
        self.power = power
        self.const = const


    def colibrate_sensor(self, numsteps=10):
        '''
        Approximates coefficients for output curve. This could be found there (page 4):
        https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a41sk_e.pdf

        numsteps : int
            amount of steps 
        '''
        distances = range(self.min, self.max + 1, (self.max - self.min) / numsteps)
        voltages = []

        output_curve = lambda x, m, a, b: m + a * (x ** b)

        for _ in distances:
            all_measurements = [
                self.mcp.read_abc(self.channel) * 1.8 * self.scale
                for _ in range(5)
            ]

            voltages.append(sum(all_measurements) / len(all_measurements))

        popt, _ = curve_fit(
            output_curve,
            np.array(voltages), np.array(distances)
        )

        self.const = popt[0]
        self.coeff = popt[1]
        self.power = popt[2]


    def measure_distance(self):
        distance = self.const
        distance += (self.mcp.read_adc(self.channel) * 1.8 * self.scale) ** self.power

        if self.min <= distance <= self.max:
            return distance
        return -1
