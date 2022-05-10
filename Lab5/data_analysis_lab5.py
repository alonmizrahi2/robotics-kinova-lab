import numpy as np
import matplotlib.pyplot as plt
import pickle
import glob
import math
import pandas as pd
import plotly.express as px

# Function that Calculate Root Mean Square
def rmsValue(arr, n):
    square = 0
    mean = 0.0
    root = 0.0

    # Calculate square
    for i in range(0, n):
        square += (arr[i] ** 2)

    # Calculate Mean
    mean = (square / (float)(n))

    # Calculate Root
    root = math.sqrt(mean)

    return root
#data_path = set your own path to data dir
data_path = '/home/eran-feingold/Desktop/study/robotics_kinova_lab/Lab5/new_data/'
records = []
for file in glob.glob(data_path + '/*/*.pkl'):
    with open(file, 'rb') as h:
        records.append(pickle.load(h))
records = records[5:]
runs = ['A Lambda=0.6','B Lambda=0.6','C Lambda=0.6','D Lambda=0.6','E Lambda=0.6']
def graph_data(runs,title):
    plt.figure(figsize=(10, 5))
    for run in range(len(runs)):
        data_dict = {'RMS':[],'iteration':[]}
        n = 0
        for i in range(len(records[run][1])):
            rms = rmsValue(records[run][1][i], len(records[run][1][i]))
            data_dict['RMS'].append(rms)
            data_dict['iteration'].append(n)
            n = n+1
        data_df = pd.DataFrame.from_dict(data_dict)
        data_df.to_csv('/home/eran-feingold/Desktop/study/robotics_kinova_lab/Lab5/plots_data/RMS_{}.csv'.format(runs[run]))
        plt.plot(data_df['iteration'], data_df['RMS'])
    plt.title(title)
    plt.xlabel('iteration')
    plt.ylabel('RMS')
    plt.grid()
    plt.legend(runs,loc=1)
    plt.savefig('/home/eran-feingold/Desktop/study/robotics_kinova_lab/Lab5/plots_data/RMS_{}.png'.format(title))

graph_data(runs,'Lambdas = 0.6')
