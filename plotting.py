import pandas as pd

import matplotlib.pyplot as plt

roll_data = pd.read_csv("roll.csv")
print(roll_data.head())
pitch_data = pd.read_csv("pitch.csv")
print(pitch_data.head())

# plt.figure(1)
roll_time = len(roll_data)
print(roll_data.columns)
print(roll_data["roll_accel"])
plt.plot(range(roll_time), roll_data["roll_accel"], label="roll_accel")
plt.plot(range(roll_time), roll_data["roll_intl"], label="roll_int")
plt.plot(range(roll_time), roll_data["roll_cal"], label="roll_cal")
plt.legend()
plt.title("Roll Data")
plt.savefig("roll.png")

plt.figure(2)
pitch_time = len(pitch_data)
plt.plot(range(pitch_time), pitch_data["pitch_accel"], label="pitch_accel")
plt.plot(range(pitch_time), pitch_data["pitch_intl"], label="pitch_int")
plt.plot(range(pitch_time), pitch_data["pitch_cal"], label="pitch_cal")
plt.legend()
plt.title("Pitch Data")
plt.savefig("pitch.png")

