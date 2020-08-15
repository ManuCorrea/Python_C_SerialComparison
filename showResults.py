"""
Built based on https://matplotlib.org/gallery/lines_bars_and_markers/barchart.html#sphx-glr-gallery-lines-bars-and-markers-barchart-py
First you have to generate data with Python and C programs with the Arduino attached
"""
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# MODIFY to make the correct title
N_EXECUTIONS = 100

# the data is saved in the same order as in labels
labels = ['orderAndAck()', 'dataRequest()', 'nResponses(20)', 'nResponses(60)', 'nResponses(100)', 'nResponses(200)']
# get python data
pythonTimes = []
with open("data/pythonData.txt", "r") as pythonFile:
    for line in pythonFile.readlines():
        pythonTimes.append(float(line.rstrip("\n")))
print(pythonTimes)
# get c data
cTimes = []
with open("data/cData.txt", "r") as cFile:
    for line in cFile.readlines():
        cTimes.append(float(line.rstrip("\n")))
print(cTimes)

x = np.arange(len(labels))  # the label locations
width = 0.35  # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(x - width/2, pythonTimes, width, label='Python')
rects2 = ax.bar(x + width/2, cTimes, width, label='C')

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Scores in milliseconds')
ax.set_title('Scores in serial tests with ' + str(N_EXECUTIONS) + " executions")
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.legend()


def autolabel(rects):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for rect in rects:
        height = rect.get_height()
        ax.annotate('{}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom')


autolabel(rects1)
autolabel(rects2)

fig.tight_layout()

plt.show()
