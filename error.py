import csv
import matplotlib.pyplot as plt

t = []
e = []

with open('/tmp/tracking_error.csv') as f:
    reader = csv.DictReader(f)
    for r in reader:
        t.append(float(r['time']))
        e.append(float(r['error']))

plt.plot(t, e)
plt.xlabel('Time (s)')
plt.ylabel('Tracking Error (m)')
plt.title('Tracking Error vs Time')
plt.grid()
plt.show()
