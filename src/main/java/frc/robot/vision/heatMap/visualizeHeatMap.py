import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np

def plot_heatmap(csv_file, multiplier=1000000, showValues=False):
    df = pd.read_csv(csv_file, index_col=0)
    df *= multiplier  # Needed if you want to see the values in each grid

    unique_values = np.unique(df.values.flatten())
    sorted_values = np.sort(unique_values)[::-1]
    second_max_value = sorted_values[1] if len(sorted_values) > 1 else np.nan
    
    vmin = df.min().min()
    vmax = second_max_value if not np.isnan(second_max_value) else df.max().max()

    plt.figure(figsize=(len(df.columns), len(df)))
    ax = sns.heatmap(df.T, cmap="RdBu", annot=showValues, fmt=".2f", linewidths=.5, vmin=vmin, vmax=vmax, square=True)
    ax.invert_xaxis()
    ax.set_xlabel("")

    # dynamic x-axis and y-axis tick labels based on DataFrame length
    ax.set_yticks(np.arange(0.5, len(df.columns), 1))
    ax.set_yticklabels(range(len(df.columns), 0, -1))
    
    ax.set_xticks(np.arange(0.5, len(df), 1))
    ax.set_xticklabels(range(len(df), 0, -1))

    plt.title(f'Vision Heatmap')
    plt.show()

if __name__ == "__main__":
    csv_file_path = "C:\\Users\\rakra\\Common\\HeatMap_0.25_0.7316420224360229__0.csv"
    
    plot_heatmap(csv_file_path)