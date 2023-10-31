import matplotlib.pyplot as plt
import pandas as pd


df = pd.read_excel('src/subharmonic_apf/data/result1 subharmonic, with sampling.xlsx')
x_data1 = df['x']
y_data1= df['y']

df = pd.read_excel('src/subharmonic_apf/data/result1 subharmonic, without sampling.xlsx')
x_data2 = df['x']
y_data2= df['y']
ob_data2=df['distance to obstacle']

df = pd.read_excel('src/subharmonic_apf/data/result1 traditional, without sampling.xlsx')
x_data3 = df['x']
y_data3= df['y']
ob_data3=df['distance to obstacle']

df = pd.read_excel('src/subharmonic_apf/data/result1 traditional, with samping.xlsx')
x_data4 = df['x']
y_data4 = df['y']
ob_data4=df['distance to obstacle']

df = pd.read_excel('src/subharmonic_apf/data/result2 traditional, with samping.xlsx')
ob_data1=df['distance to obstacle']


plt.figure()

plt.plot(x_data1, y_data1, color='red', alpha=0.8, linewidth=3, label='subharmonic, with sampling')
plt.plot(x_data2, y_data2, color='green', alpha=0.8, linewidth=3, label='subharmonic, without sampling')
plt.plot(x_data3, y_data3, color='blue', alpha=0.8, linewidth=3, label='traditional, without sampling')
plt.plot(x_data4, y_data4, color='yellow', alpha=0.8, linewidth=3, label='traditional, with sampling')

# plt.scatter(x_data1, y_data1, c='red', s=50, label='legend')
# plt.scatter(x_data2, y_data2, c='blue', s=50, label='legend')
# plt.scatter(x_data3, y_data3, c='green', s=50, label='legend')
#plt.scatter(x_data4, y_data4, c='yellow', s=10, label='legend')


plt.xlabel(" X-axis position", fontdict={'size': 16})
plt.ylabel(" Y-axis position", fontdict={'size': 16})
plt.title("Trajectory", fontdict={'size': 20})
plt.legend(loc='best')
plt.savefig('trajectory.png')
plt.show()

#==================================================================================================
plt.figure()

plt.plot(ob_data1, color='red', alpha=0.8, linewidth=3, label='subharmonic, with sampling')
plt.plot(ob_data2, color='green', alpha=0.8, linewidth=3, label='subharmonic, without sampling')
plt.plot(ob_data3, color='blue', alpha=0.8, linewidth=3, label='traditional, with sampling')
plt.plot(ob_data4, color='yellow', alpha=0.8, linewidth=3, label='traditional, without sampling')

# plt.scatter(x_data1, y_data1, c='red', s=50, label='legend')
# plt.scatter(x_data2, y_data2, c='blue', s=50, label='legend')
# plt.scatter(x_data3, y_data3, c='green', s=50, label='legend')
#plt.scatter(x_data4, y_data4, c='yellow', s=10, label='legend')


plt.ylabel(" distance/m", fontdict={'size': 16})
plt.title("Distance to Obstacle", fontdict={'size': 20})
plt.legend(loc='best')
plt.savefig('dis.png')

plt.show()
