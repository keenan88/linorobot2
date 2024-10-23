import numpy as np
import struct

# Sample data from your output, ensuring to include enough pixels
# You should replace this with the actual depth image data
data = [
     64,
    48,
    181,
     63,

    0, 0, 128, 127,  # Pixel 1
    0, 0, 128, 127,  # Pixel 2
    0, 0, 128, 127,  # Pixel 3
    0, 0, 128, 127,  # Pixel 4
    # Add more pixels as needed up to width * height * 4
]

for i in range(0, len(data), 4):

    four_bytes = data[i] << 24 | data[i+1] << 16 | data[i+2] << 8 | data[i+3]

    # print(bin(data[i]))
    # print(bin(data[i+1]))
    # print(bin(data[i+2]))
    # print(bin(data[i+3]))

    print(bin(four_bytes))
