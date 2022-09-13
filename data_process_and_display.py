from time import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.fftpack import fft
from scipy.signal import butter, lfilter, freqz
from tkinter import filedialog
import os

#Low-pass filter functions
def butter_lowpass(cutoff, fs, order=5):
    return butter(order, cutoff, fs=fs, btype='low', analog=False)

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

#Prompt user to open every necessary file
print("Please open the ACCELERATION data file!")
accel_file_path = filedialog.askopenfilename(filetypes=[("text",".txt")]) 
print("Please open the TIMESTAMPS data file!")
timestamps_file_path = filedialog.askopenfilename(filetypes=[("text",".txt")])

try:

    with open(accel_file_path, 'r') as f:
        content_list = f.readlines()[1:] #Reads from 2nd line onwards
        new_list = [x[:-1] for x in content_list] #Removes \n from the list
        accel_list = [round(float(item), 20) for item in new_list] #Rounds the whole list with 20 decimal cases to prevent data loss due to a bug generating the plot

except:
    print("Something went wrong while reading from the accel file!")

file_name = os.path.basename(accel_file_path)
size = len(file_name)
# Slice string to remove initial and last 4 (.txt) characters from string
mod_string = file_name[11:size - 21]
print(mod_string)

try:

    with open(timestamps_file_path, 'r') as f:
        content_list = f.readlines()[1:] #Reads from 2nd line onwards
        new_list = [x[:-1] for x in content_list] #Removes \n from the list
        timestamps_list = [round(float(item), 20) for item in new_list] #Rounds the whole list with 20 decimal cases to prevent data loss due to a bug generating the plot

except:
    print("Something went wrong while reading from the timestamps file!")


N = len(accel_list) #Number of samples
freq = N/(float(timestamps_list[N-1])-float(timestamps_list[0]))  #sample rate (Hz)
T = 1/freq #Period


# Filter requirements.
order = 6 # Filter order
cutoff = 500  # desired cutoff frequency of the filter, Hz

# Filter the data, and plot both the original and filtered signals.
accel_list_filt = butter_lowpass_filter(accel_list, cutoff, freq, order)


# Plot Data
plt.figure(figsize=(7.5, 2.5)) #Set figure size
plt.plot(timestamps_list, accel_list_filt) #Plot amplitude vs timestamps
#plt.xlim(0,timestamps_list[len(timestamps_list)-1]) #Set X-limit
#plt.xlim(0,) #Set Y-limit
plt.xlabel("Time (s)") #Set X label
plt.ylabel('Amplitude (g)') #Set Y label
plt.title('Z AXIS ACCELERATION') #Set plot title
plt.grid() #Adds a grid to the plot
#plt.savefig(mod_string + '_chart.png', dpi=300, bbox_inches='tight') #Saves the figure

#Compute and plot FFT
plt.figure(figsize=(7.5, 2.5)) #Set figure size
xf = np.linspace(0.0, 1.0/(2.0*T), int(N/2)) #Evenly spaces the timestamps until half the sample size (Nysquish theory)
yf = fft(accel_list_filt) #Perform FFT on the data
yf[0] = 0 #Set first value to 0 to prevent FFT bug
plt.plot(xf, 2.0/N * np.abs(yf[0:int(N/2)])) #Plot FFT
#plt.xlim(0, 500) #Set X-limit
#plt.ylim(0,) #Set Y-limit
plt.xlabel('Frequency (Hz)') #Set X label
plt.ylabel('Amplitude (g)') #Set Y label
plt.title('Z AXIS FFT') #Set plot title
plt.grid() #Adds a grid to the plot
#plt.savefig(mod_string + '_FFT.png', dpi=300, bbox_inches='tight') #Saves the figure


#Plot spectrogram
plt.figure(figsize=(7.5, 2.5)) #Set figure size
plt.specgram(accel_list_filt, Fs=freq, mode='magnitude', noverlap=128, NFFT=2**11) #Perform spectrogram on the data
#plt.ylim(0, 500) #Set Y-limit
plt.xlabel('Time (s)') #Set X label
plt.ylabel('Frequency (Hz)') #Set Y label
plt.title('Z AXIS SPECTROGRAM') #Set plot title
#plt.savefig(mod_string + '_spectrogram.png', dpi=300, bbox_inches='tight') #Saves the figure

plt.show() #Show all plots

