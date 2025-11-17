import matplotlib.pyplot as plt
from pathlib import Path
import csv
import numpy as np


def read_csv(path):
    seq = []
    rtt = []
    with open(path, newline='') as file:
        reader = csv.DictReader(file)
        for row in reader:
            seq.append(int(row['seq']))
            rtt.append(float(row['rtt_ms']))
    # Return as np array if we want to do calculations later
    return np.array(seq), np.array(rtt)

files = Path('~/ros2_logs/').expanduser()

qosMap = {'results_1': 'reliable_volatile', 'results_2': 'best_effort_volatile', 'results_3': 'reliable_transient_local', 'results_4': 'best_effort_transient_local', 'results_5': 'reliable_keep_all', 'results_6': 'best_effort_keep_all'}

for f in files.iterdir():
    if f.suffix != '.csv':
        continue

    # Find the title name from qosMap
    plot_title = None
    for key, title in qosMap.items():
        if key in f.name:
            plot_title = title
            break
    
    seq, rtt = read_csv(f)
    
    #Plot RTT on y-axis and sequence number on x-axis
    plt.figure()
    plt.plot(seq, rtt)
    plt.xlabel('Sequence Number')
    plt.ylabel('RTT (ms)')
    plt.title(plot_title)
    plt.grid(True)
    plt.tight_layout()
    plt.show()

"""
IN PROGRESS
pairs = [
    ('results_1', 'results_2'), # reliable_volatile vs best_effort_volatile
    ('results_5', 'results_6') # reliablie_keep_all vs best_effort_keep_all
]

colors = ['tab:blue', 'tab:orange']

for pair in pairs:
    plt.figure()
    for i,key in enumerate(pair):
        csv_file = next((f for f in files.iterdir() if f.suffix == '.csv' and key in f.name), None)
        if csv_file == None:
            continue

        seq, rtt = read_csv(csv_file)
        title = qosMap.get(key)
        plt.plot(seq,rtt, label=title, color=colors[i])

        plt.xlabel('Sequence Number')
        plt.ylabel('RTT (ms)')
        pair_titles = [qosMap.get(k, k) for k in pair]
        plt.title(f'Comparison: {pair_titles[0]} vs {pair_titles[1]}')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()
    
"""
