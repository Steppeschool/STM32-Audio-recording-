from gettext import npgettext
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation 
import serial
import struct
from threading import Thread

# uart comport
uart_mcu = serial.Serial('COM7', 115200, parity=serial.PARITY_NONE, \
    stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
# number of frequencies
N = 32

# defining plot parameters
fig = plt.figure(figsize=(6,6))
ax1 = plt.subplot(111, projection='polar')
theta = np.linspace(0, 2*np.pi, N)
fft_values = np.zeros(N)
fmt = "%df"% (N)
# creating thread to read the uart
def read_uart():
    global fft_values
    print("running thread")
    # reference string (">>>>") to indicate the end of the data packet
    while True:
        data_uart = uart_mcu.read_until(b'\xf9\xf8xN')
        # convert the data to float
        if data_uart.__len__() == N * 4 + 4:
            fft_values = struct.unpack(fmt, np.flip(data_uart[0:(N * 4)]))

# animaption function to update the plot
def plot_animation(i):
    global fft_values
    # clearing the figure
    ax1.clear()
    #ax1.set_rscale('symlog')
    ax1.set_xticklabels(['0 Hz', '2000 Hz', '4000 Hz','6000 Hz', '8000'\
        , '10000 Hz', '12000 Hz', '14000 Hz'])
    ax1.set_rlim(0, 2500)
    ax1.bar(theta, fft_values, width=0.2, bottom=0.0)

ani = FuncAnimation(fig, plot_animation, frames= 100, interval = 10, blit=False)

if __name__ == "__main__":
    t1 = Thread(target=read_uart, daemon=True)
    t1.start()
    plt.show()
