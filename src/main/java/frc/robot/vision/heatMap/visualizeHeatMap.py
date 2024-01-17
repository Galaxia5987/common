import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

def plot_heatmap(csv_file):
    df = pd.read_csv(csv_file, index_col=0)

    plt.figure(figsize=(10, 8))
    sns.heatmap(df, cmap="YlGnBu", annot=True, fmt=".2f", linewidths=.5)
    plt.title('Vision HeatMap')
    plt.show()

if __name__ == "__main__":
    csv_file_path = "path" #replace with path
    plot_heatmap(csv_file_path)