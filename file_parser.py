import matplotlib.pyplot as plt
import serial
import numpy as np
from timeit import default_timer as timer
import datetime
from scipy.fftpack import fft


SSF = 2048 #Sensitivity Scale Factor for ACCEL_FS_SEL = 3 (Â±16g)

time_s = 1 #Logging time

header_lsb = b'\x55' #Header byte for verification

serialPort = serial.Serial(port = "COM5", baudrate=921600, bytesize=8, timeout=2) #Initializes and configures UART comm

#Get the current time to write on the 1st line of every generated file
file_1st_line = datetime.datetime.now().strftime("%d/%m/%Y - %H:%M:%S") 

#UART LOGGER FILE NAME 
ul_file_name = 'uart_logger_' + 'engine_on_coolingfan_on_extractor_on_final_test' + '_Z_' + datetime.datetime.now().strftime("%d_%m_%Y-%H.%M") + '.bin'

#TIMESTAMPS FILE NAME 
ts_file_name = 'timestamps_' + 'engine_on_coolingfan_on_extractor_on_final_test' + '_Z_' + datetime.datetime.now().strftime("%d_%m_%Y-%H.%M") + '.txt'

#ACCELERATION DATA FILE NAME 
ad_file_name = 'accel_data_' + 'engine_on_coolingfan_on_extractor_on_final_test' + '_Z_' + datetime.datetime.now().strftime("%d_%m_%Y-%H.%M") + '.txt'

# Log UART during time_s
try:

    with open(ul_file_name, 'wb') as f: #Opens uart logger file to write as binary
    
        time_plot = [] #timestamps list 
        start_time = timer() #starts the timer
        stop_time = start_time + float(time_s) 

        while timer() < stop_time: #log for the selected time
            a = serialPort.read()
            f.write(a)

            # Find first header 0x55
            if a == b'\x55':
                # Prevents time append when data value is 55  
                f.write(serialPort.read(2)) #Reads the 2 corresponding acceleration data bytes
                time_plot.append(timer()-start_time) #Appends the timestamp to the list

except:
    print("Something went wrong while typing into the Uart Logger file!") 


try:
    with open(ul_file_name, 'rb+') as f: #Opens uart logger file to read as binary

        loop_end = False
                    
        #For plotting
        data_plot = [] #Acceleration data list
        while not loop_end: #Until it reaches the end of the file

            # Find first header 0x55 
            h1 = f.read(1) 
           
            if h1 == header_lsb :

                raw_data_b = f.read(2)

                if raw_data_b == b'' or len (raw_data_b) != 2:
                    time_plot.pop() #Removes the last value from the list to prevent any incorrent value
                    loop_end = True
                    break # end loop now as we reached end of file
                else:
                    raw_data = int.from_bytes(raw_data_b, "big", signed=True)
                    accel = raw_data/SSF #Converts raw data to g units using the SSF for the selected Full-Scale Range

                    data_plot.append(accel) #Appends the aceleration data to the list
            
            elif h1 == b'' :
                # No remainig data in the file, end has been reached
                 loop_end = True

except:
    print("Something went wrong while reading from the file!")

#Saves the timestamps data to a .txt file
try:
    with open(ts_file_name, 'w') as f:
        f.write('TIMESTAMPS -> ' + file_1st_line + '\n')
        for d in time_plot:
            f.write(f"{d}\n")
except:
    print("Something went wrong while writing to the timestamps file!")


#Saves the acceleration data to a .txt file
try:
    with open(ad_file_name, 'w') as f:
        f.write('ACCELERATION DATA -> ' + file_1st_line + '\n')
        for d in data_plot:
            f.write(f"{d}\n")
except:
    print("Something went wrong while writing to the accel file!")


