import serial

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM5'
ser.open()

in_seconds = 5
f = open('c:\\temp\\audio_raw', 'wb')
while (1):
    

        if (ser.inWaiting()):
            in_data =ser.read(size = 32768)
            print(len(in_data), type(in_data))
            if ((in_seconds > 0) and (len(in_data)>1000)):
                f.write(in_data)
                # print(in_data)
                in_seconds = in_seconds - 1
            
        if (in_seconds == 0):
            f.close()
            print('file closed')
            break

ser.close()
