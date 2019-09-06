""" Credits ozzmaker.com and Dorsic """


import smbus
import time
import datetime
import sys
from apscheduler.schedulers.background import BlockingScheduler

class PowerMode():
    SLEEP = 0x00
    FORCED = 0x01
    NORMAL = 0x03

class StandbyTime(object):
    """ Sets the st_b register. """
    TMS_0_5 = 0x00
    TMS_62_5 = 0x20
    TMS_125 = 0x40
    TMS_250 = 0x60
    TMS_500 = 0x80
    TMS_1000 = 0xA0
    TMS_2000 = 0xC0
    TMS_4000 = 0xE0

class PressureResolution(object):
    """ Sets the osrs_p register. """
    SKIPPED = 0x00 # 0x80000
    ULTRA_LOW_POWER = 0x04
    LOW_POWER = 0x08
    STANDARD = 0x0C
    HIGH = 0x10
    ULTRA_HIGH = 0x14

class TemperatureResolution(object):
    """ Sets the osrs_t register.
        Do not use setting higher than LOW_POWER as they do not contribute to finer pressure reading.
    """
    SKIPPED = 0x00 # 0x80000
    ULTRA_LOW_POWER = 0x20
    LOW_POWER = 0x40
    STANDARD = 0x60
    HIGH = 0x80
    ULTRA_HIGH = 0xE0

class IIRFilter(object):
    OFF = 0x00
    COEF_2 = 0x08
    COEF_4 = 0x10
    COEF_8 = 0x18
    COEF_16 = 0x1C

class BMP280(object):

    def __init__(self, i2cdev=1, i2caddress=0x77):
        # Get I2C bus
        # BMP280 address, 0x77
        self.bus = smbus.SMBus(i2cdev)
        self.address = i2caddress
        self.b1 = None

    def _convertTemp(self, blockdata, adc_t):
        # Convert the data
        # Temp coefficents
        dig_T1 = blockdata[1] * 256 + blockdata[0]
        dig_T2 = blockdata[3] * 256 + blockdata[2]
        if dig_T2 > 32767 :
            dig_T2 -= 65536
        dig_T3 = blockdata[5] * 256 + blockdata[4]
        if dig_T3 > 32767 :
            dig_T3 -= 65536
        # Temperature offset calculations
        var1 = ((adc_t) / 16384.0 - (dig_T1) / 1024.0) * (dig_T2)
        var2 = (((adc_t) / 131072.0 - (dig_T1) / 8192.0) * ((adc_t)/131072.0 - (dig_T1)/8192.0)) * (dig_T3)
        t_fine = (var1 + var2)
        cTemp = (var1 + var2) / 5120.0
        return cTemp, t_fine

    def _convertPreassure(self, blockdata, t_fine, adc_p):
        # Pressure coefficents
        dig_P1 = blockdata[7] * 256 + blockdata[6]
        dig_P2 = blockdata[9] * 256 + blockdata[8]
        if dig_P2 > 32767 :
            dig_P2 -= 65536
        dig_P3 = blockdata[11] * 256 + blockdata[10]
        if dig_P3 > 32767 :
            dig_P3 -= 65536
        dig_P4 = blockdata[13] * 256 + blockdata[12]
        if dig_P4 > 32767 :
            dig_P4 -= 65536
        dig_P5 = blockdata[15] * 256 + blockdata[14]
        if dig_P5 > 32767 :
            dig_P5 -= 65536
        dig_P6 = blockdata[17] * 256 + blockdata[16]
        if dig_P6 > 32767 :
            dig_P6 -= 65536
        dig_P7 = blockdata[19] * 256 + blockdata[18]
        if dig_P7 > 32767 :
            dig_P7 -= 65536
        dig_P8 = blockdata[21] * 256 + blockdata[20]
        if dig_P8 > 32767 :
            dig_P8 -= 65536
        dig_P9 = blockdata[23] * 256 + blockdata[22]
        if dig_P9 > 32767 :
            dig_P9 -= 65536
        # Pressure offset calculations
        var1 = (t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * (dig_P6) / 32768.0
        var2 = var2 + var1 * (dig_P5) * 2.0
        var2 = (var2 / 4.0) + ((dig_P4) * 65536.0)
        var1 = ((dig_P3) * var1 * var1 / 524288.0 + ( dig_P2) * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * (dig_P1)
        p = 1048576.0 - adc_p
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = (dig_P9) * p * p / 2147483648.0
        var2 = p * (dig_P8) / 32768.0
        pressure = (p + (var1 + var2 + (dig_P7)) / 16.0) / 100
        return pressure

    def readData(self):
        # Read data back from 0x88(136), 24 bytes
        b1 = self.bus.read_i2c_block_data(self.address, 0x88, 24)
        # BMP280 address, 0x77(118)
        # Select Control measurement register, 0xF4(244)
        #		0x27(39)	Pressure and Temperature Oversampling rate = 1
        #					Normal mode
        self.bus.write_byte_data(self.address, 0xF4, 0x27)
        # BMP280 address, 0x77(118)
        # Select Configuration register, 0xF5(245)
        #		0xA0(00)	Stand_by time = 1000 ms
        self.bus.write_byte_data(self.address, 0xF5, 0xA0)

        time.sleep(0.01)
        # BMP280 address, 0x77(118)
        # Read data back from 0xF7(247), 8 bytes
        # Pressure MSB, Pressure LSB, Pressure xLSB, Temperature MSB, Temperature LSB
        # Temperature xLSB, Humidity MSB, Humidity LSB
        data = self.bus.read_i2c_block_data(0x77, 0xF7, 8)

        # Convert pressure and temperature data to 19-bits
        adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16
        adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16

        temp = self._convertTemp(b1, adc_t)
        press = self._convertPreassure(b1, temp[1], adc_p)
        return temp[0], press

    def init(self, power_mode=PowerMode.NORMAL, 
                    standby_time=StandbyTime.TMS_0_5,
                    temp_resolution=TemperatureResolution.ULTRA_LOW_POWER, 
                    pressure_resolution=PressureResolution.ULTRA_HIGH, 
                    iir=IIRFilter.OFF):
        # Read data back from 0x88(136), 24 bytes
        self.b1 = self.bus.read_i2c_block_data(self.address, 0x88, 24)
        # BMP280 address, 0x77(118)
        # Select Control measurement register, 0xF4(244)
        #		0x27(00111001)	Pressure and Temperature Oversampling rate = 1
        #					Normal mode
        #       0X57(01010111)  Temp Oversampling rate 2x, Pressure Oversampling rate 16x, Normal mode
        self.bus.write_byte_data(self.address, 0xF4, temp_resolution+pressure_resolution+power_mode)  # 0x57
        # BMP280 address, 0x77(118)
        # Select Configuration register, 0xF5(245)
        #		0xA0(101)	Stand_by time = 1000 ms
        #       0x00(000)   Stand_by time = 0.5 ms
        #       0x1E(00011110)  Stand_by time = 0.5 ms, IIR filter 16x, I2C interface
        self.bus.write_byte_data(self.address, 0xF5, standby_time+iir+0x10)

    def read(self):
        # BMP280 address, 0x77(118)
        # Read data back from 0xF7(247), 8 bytes
        # Pressure MSB, Pressure LSB, Pressure xLSB, Temperature MSB, Temperature LSB
        # Temperature xLSB, Humidity MSB, Humidity LSB
        data = self.bus.read_i2c_block_data(0x77, 0xF7, 8)

        # Convert pressure and temperature data to 19-bits
        adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16
        adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16

        temp = self._convertTemp(self.b1, adc_t)
        press = self._convertPreassure(self.b1, temp[1], adc_p)
        return temp[0], press

def logData(f=None):
    global s
    # init and read
    data = s.read()
    if f:
        f.write("{0}\t {1:.4f}\t {2:.4f}\n".format(time.time(), data[0], data[1]))
    else:
        print("{0}\t {1:.4f}\t {2:.4f}".format(time.time(), data[0], data[1]))

def fileName():
    return '/home/pi/imu/P' + str(time.time()).split('.')[0]+".csv"

s = BMP280()
s.init(standby_time=StandbyTime.TMS_125, power_mode=PowerMode.NORMAL, 
    iir=IIRFilter.OFF, pressure_resolution=PressureResolution.ULTRA_HIGH, 
    temp_resolution=TemperatureResolution.LOW_POWER)
#scheduler = BlockingScheduler()
#job = scheduler.add_job(data, 'interval', seconds=0.01)

try:
    f = open(fileName(), "w") if len(sys.argv)>1 and sys.argv[1] == '-f' else None
    #    scheduler.start()
    while True:
        logData(f)
        #time.sleep(0.0080)
except (KeyboardInterrupt, SystemExit):
    f.close()
    pass

