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
    plt.show()

if __name__ == "__main__":
    csv_file_path = "path" #replace with path
    plot_heatmap(csv_file_path)