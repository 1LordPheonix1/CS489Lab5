import numpy as np
import csv
from scipy import interpolate

# column 0  x
# column 1  y
# column 2 yaw
# columnd 3 speed
data = None
with open('waypoints.csv', newline='') as csv_file:
    reader = csv.reader(csv_file)
    next(csv_file)  # discard header
    data = list(reader)

x1 = [float(row[0]) for row in data]
y1 = [float(row[1]) for row in data]

I = interpolate.splev(x1, y1)

# print(I)