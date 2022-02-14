from cv2 import FlannBasedMatcher
import serial
import numpy as np
from scipy.io.wavfile import write

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM5'
ser.open()

in_seconds = 5
first_time = True

while (1):
    

        if (ser.inWaiting()):
            in_data =ser.read(size = 32768)
            print(len(in_data), type(in_data))
            if ((in_seconds > 0) and (len(in_data)>1000)):
                if(first_time):
                    input_bytes=np.frombuffer(in_data, dtype=np.dtype('>i2')) 
                    first_time = False
                else:
                    input_2_bytes = np.frombuffer(in_data, dtype=np.dtype('>i2'))
                    input_bytes = np.append(input_bytes, input_2_bytes)
                print(input_bytes.shape)
                print(input_bytes.dtype)
                in_seconds = in_seconds - 1
            
        if (in_seconds == 0):
            print('file closed')
            break


ser.close()
write("c:\\temp\\output2.wav", 16000, input_bytes)
print('done')