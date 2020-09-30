import serial
from time import sleep

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from pyqtgraph.ptime import time
import serial
from scipy import signal
import numpy as np

# def twos_complement(value, bit_width):
#     if value >= 2 ** bit_width:
#         # This catches when someone tries to give a value that is out of range
#         raise ValueError("Value: {} out of range of {}-bit value.".format(value, bit_width))
#     else:
#         return value - int((value << 1) & 2 ** bit_width)


class BMD101:
    # Data A(8bytes): [AA, AA, 04, 80, 02, XX, XX, checkSum]
    def __init__(self, port, baud):
        self.t_list = []
        self.sensorStatus = 'off'
        self.isDataValid = False
        self.isDeviceReady = False
        self.HeartRate = 0.0
        self.RawDataA = b''
        self.parsedDataA = 0
        self.RawDataB = b''
        self.byteObj = 0
        try:
            self.my_serial = serial.Serial(port, baudrate=baud, timeout=0)
            sleep(1)
        except KeyboardInterrupt:
            self.my_serial.close()
        except serial.SerialException:
            raise serial.SerialException

    def parse_data(self):
        if self.my_serial.read(1) == b'\xaa' and self.my_serial.read(1) == b'\xaa':
            judgeDataType = self.my_serial.read(1)
            if judgeDataType == b'\x04':
                self.RawDataA = bytearray(self.my_serial.read(5)) # array includes 4 bytes dataA and 1 byte checksum.
                check_sum_received = self.RawDataA.pop()    # get last element(checksum) then remove it from array.
                data_sum = 0
                for val in self.RawDataA:
                    data_sum += val
                checksum_calc = ~(data_sum & 0xff) & 0xff   # inverse of lowest 8 bit of data_sum.
                if check_sum_received == checksum_calc:
                    self.isDataValid = True
                    data_a = ((self.RawDataA[2] << 8) + self.RawDataA[3])
                    self.parsedDataA = data_a & 0xffff  # take lowest 16 bits as valid data.
                    if data_a >= 32768:
                        self.parsedDataA = data_a - 65536
                    else:
                        self.parsedDataA = data_a
                else:
                    self.isDataValid = False
                check_sum_received = 0
                checksum_calc = 0

            # if judgeDataType == b'\x12':
            #     self.RawDataB = bytearray(self.my_serial.read(19))  # array includes 18 bytes dataA and 1 byte checksum.
            #     print(self.RawDataB)
            #     check_sum_received = self.RawDataB.pop()  # get last element(checksum) then remove it from array.
            #     data_sum = 0
            #     for val in self.RawDataB:
            #         data_sum += val
            #     checksum_calc = ~(data_sum & 0xff) & 0xff    # inverse of lowest 8 bit of data_sum.
            #     if check_sum_received == checksum_calc:
            #         print('Pass B')
            #     #     self.isDataValid = True
            #     #     self.HeartRate = self.RawDataA[3]
            #     #     # print("HR=", self.HeartRate)
            #     # else:
            #     #     self.isDataValid = False
            #     check_sum_received = 0
            #     checksum_calc = 0



app = QtGui.QApplication([])
p = pg.plot()
p.setWindowTitle('live plot from serial')
curve = p.plot()

my_ecg = BMD101('COM4', 57600)

data = [0]
ptr = 0


def Implement_Notch_Filter(fs, band, freq, ripple, order, filter_type, data):
    # Required input defintions are as follows;
    # fs:   sampling rate
    # band:   The bandwidth around the centerline freqency that you wish to filter
    # freq:   The centerline frequency to be filtered
    # ripple: The maximum passband ripple that is allowed in db
    # order:  The filter order.  For FIR notch filters this is best set to 2 or 3,
    #         IIR filters are best suited for high values of order.  This algorithm
    #         is hard coded to FIR filters
    # filter_type: 'butter', 'bessel', 'cheby1', 'cheby2', 'ellip'
    # data:         the data to be filtered
    nyq  = fs/2.0
    low  = freq - band/2.0
    high = freq + band/2.0
    low  = low/nyq
    high = high/nyq
    b, a = signal.iirfilter(order, [low, high], rp=ripple, btype='bandstop',
                     analog=False, ftype=filter_type)
    filtered_data = signal.lfilter(b, a, data)
    return filtered_data


temp = []
def update():
    global curve, data, ptr
    my_ecg.parse_data()
    data.append(float(my_ecg.parsedDataA))
    xdata = np.array(data, dtype='float64')
    curve.setData(xdata)
    ptr += 1
    app.processEvents()


timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(1)

if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()



