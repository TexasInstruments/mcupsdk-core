import os
import sys
import matplotlib.pyplot as plt

col0 = "red"
col1 = "blue"
col2 = "green"
col3 = "pink"

f_name = sys.argv[1]
f_size = os.path.getsize(f_name)

f = open(f_name, "rb")

data0 = f.read(int(f_size/4))
data1 = f.read(int(f_size/4))
data2 = f.read(int(f_size/4))
data3 = f.read(int(f_size/4))

x0 = []
y0 = []

for tx in range(0, 128):
	for rx in range(0, 128):
		idx = 128*tx + rx

		if(int(data0[idx]) == 1):
			x0.append(tx)
			y0.append(rx)
			print(tx)
			print(rx)

x1 = []
y1 = []

for tx in range(0, 128):
	for rx in range(0, 128):
		idx = 128*tx + rx
		if(int(data1[idx]) == 1):
			x1.append(tx)
			y1.append(rx)


x2 = []
y2 = []

for tx in range(0, 128):
	for rx in range(0, 128):
		idx = 128*tx + rx
		if(int(data2[idx]) == 1):
			x2.append(tx)
			y2.append(rx)


x3 = []
y3 = []

for tx in range(0, 128):
	for rx in range(0, 128):
		idx = 128*tx + rx
		if(int(data3[idx]) == 1):
			x3.append(tx)
			y3.append(rx)

plt.scatter(x0,y0,c=col0,linewidths=2)
plt.scatter(x1,y1,c=col1,linewidths=2)
plt.scatter(x2,y2,c=col2,linewidths=2)
plt.scatter(x3,y3,c=col3,linewidths=2)


plt.xlabel("TX")
plt.ylabel("RX")

plt.xlim(0,128)
plt.ylim(0,128)

plt.show()
