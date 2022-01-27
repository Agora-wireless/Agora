import sys
import pandas as pd
from matplotlib import pyplot as plt
if len(sys.argv) != 2:
  print('usage: python3', str(sys.argv[0]), '<data_file>')
  exit()
columns = ["Frame", "EVM", "SNR"]
data = pd.read_csv(sys.argv[1], usecols=columns)
plt.subplot(2, 1, 1)
plt.plot(data.Frame, data.SNR)
plt.subplot(2, 1, 2)
plt.plot(data.Frame, data.EVM)
plt.show()
