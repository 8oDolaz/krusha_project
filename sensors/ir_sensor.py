from turtle import distance
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn


class MCP:
    '''
    Analogut to digital transformer
    MCP3008 for our robot
    '''
    def __init__(self):
        spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
        cs = digitalio.DigitalInOut(board.D5)

        self.mcp = MCP.MCP3008(spi, cs)


class Sharp_IR:
    def __init__(self, x, y, theta, mcp, mcp_channel,
                 min=4, max=15):
        '''
        x, y, theta : int, int, int
            position relative to robot

        mcp : MCP (implemented above)
            ADC to read data

        mcp_channel : int
            channel 0..7 to ADC to read data from

        min, max : int, int
            mininal and maximal threshold of mesuarements

        coeffs : list of floats
            coeffs of a polynomial used to approximate analogue output to cm
        '''
        self.x, self.y, self.theta = x, y, theta

        if not 0 <= mcp_channel <= 7:
            raise ValueError('Invalid mcp channel provided. Option are: 0..7')
        channels_map = [
            MCP.P0, MCP.P1, MCP.P2, MCP.P3,
            MCP.P4, MCP.P5, MCP.P6, MCP.P7,
        ]

        self.mcp = AnalogIn(mcp, channels_map[mcp_channel])

        self.min, self.max = min, max

        self.coeffs = [
            -2.70585774e-10, 4.55301583e-08, -3.19212470e-06,
            1.20701093e-04, -2.65057588e-03, 3.34831648e-02,
            -2.15281742e-01, 3.20088948e-01, 3.29955408e+00
        ]


    def measure_distance(self):
        raw_data = self.mcp.voltage

        distance = 0
        for i, coeff in enumerate(self.coeffs[::-1]):
            distance += coeff * (raw_data ** i)

        if self.min <= distance <= self.max:
            return distance
        return -1
