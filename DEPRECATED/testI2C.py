import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

i2c=busio.I2C(board.SCL,board.SDA)
ads = ADS.ADS1115(i2c)
chan = AnalogIn(ads,0)
chan1 = AnalogIn(ads,1)
while True:
    print("VOLTAGE1:",chan.voltage)
    print("VOLTAGE2:",chan1.voltage)