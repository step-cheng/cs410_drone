import pandas as pd

import matplotlib.pyplot as plt

data1 = pd.read_csv("w8m2.csv")
# print(data.head())
# print(data.columns)

times1 = len(data1)
# times2 = len(data2)
# times3 = len(data3)
# fig, (ax1,ax2,ax3) = plt.subplots(nrows=3, ncols=1, sharey=True, sharex=True)
plt.plot(range(500,times1), data1['yaw'][500:])

# plt.plot(range(times1), 10*data1["yaw_desired"][:times1], label="yaw_desired")
# plt.plot(range(times1), 10*data1["yaw_gyro"][:times1], label="yaw_gyro")
# plt.plot(range(times1), data1["m1"][:times1], label="m_ccw")
# plt.plot(range(times1), data1["m2"][:times1], label="m_cc")
# ax2.plot(range(times2), 100*data2["pitch_raw"][:times2], label="pitch_raw")
# ax2.plot(range(times2), 100*data2["pitch_filter"][:times2], label="pitch_filter")
# ax2.plot(range(times2), data2["motor_front"][:times2], label="motor_front")
# ax2.plot(range(times2), data2["motor_back"][:times2], label="motor_back")
# ax3.plot(range(times3), 100*data3["pitch_raw"][:times3], label="pitch_raw")
# ax3.plot(range(times3), 100*data3["pitch_filter"][:times3], label="pitch_filter")
# ax3.plot(range(times3), data3["motor_front"][:times3], label="motor_front")
# ax3.plot(range(times3), data3["motor_back"][:times3], label="motor_back")
plt.legend()
plt.title("Week 8 Milestone 2")
# ax2.set_title("Week 4 Milestone 5 OSR2")
# ax3.set_title("Week 4 Milestone 5 NoFilter")
plt.savefig("w8m2.png")
plt.show()
