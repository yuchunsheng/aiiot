
import numpy as np
import scipy.io.wavfile

try:
    with open('c:\\temp\\audio_raw', "rb") as f:
        input_bytearray= np.fromfile(f,dtype=np.dtype('<i2'))  # big endian >i2

    print(input_bytearray.shape)
    print(input_bytearray.dtype)
    scipy.io.wavfile.write("c:\\temp\\output1.wav", 16000, input_bytearray)
except IOError:
    print('Error While Opening the file!')


