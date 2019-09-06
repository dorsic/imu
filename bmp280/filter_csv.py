import numpy as np
import time
from KalmanFilter import *

in_file = "/Users/dorsic/Development/imu/P1557129366.csv"
out_file = "/Users/dorsic/Development/imu/P1557129366_kf.csv"
if __name__ == "__main__":
    kf = Bpm280Kalman()

    with open(in_file, "r") as fs:
        with open(out_file, "w") as fo:
            st = time.time()
            for line in fs:
                data = line.split("\t")
                if len(data)>2:
                    p = kf.update(np.matrix([[float(data[2])]]))
                    fo.write(data[0] + "\t" + data[1] + "\t" + str(p[0,0]) + "\n")
            print("DONE in {0:.0f} s.".format(time.time()-st))
                