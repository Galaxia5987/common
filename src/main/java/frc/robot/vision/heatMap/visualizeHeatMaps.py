import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np

def getHeightFromFile(filename):
    heightPart = ''.join(c for c in filename[:-4] if c.isdigit() or c == '.')
    return heightPart[:4]

def plotHeatmaps(directory, multiplier=1000000, showValues=False):
    csvFiles = [file for file in os.listdir(directory) if file.endswith(".csv")]

    numPlots = len(csvFiles)
    numCols = min(3, numPlots) #adjust as needed
    numRows = (numPlots - 1) // numCols + 1
    
    fig, axes = plt.subplots(numRows, numCols, figsize=(15, 5 * numRows))

    for i, csvFile in enumerate(csvFiles):
        filePath = os.path.join(directory, csvFile)

        df = pd.read_csv(filePath, index_col=0)
        
        values = df.values
        mask = values < 1
        dfAverage = np.sum(values[mask]) / np.count_nonzero(mask)
        
        df *= multiplier  #needed to see the values in each grid
        
        if numRows == 1:
            ax = axes[i % numCols]
        else:
            ax = axes[i // numCols, i % numCols]

        uniqueValues = np.unique(df.values.flatten())
        sortedValues = np.sort(uniqueValues)[::-1]
        secondMaxValue = sortedValues[1] if len(sortedValues) > 1 else np.nan
        
        vmin = df.min().min()
        vmax = secondMaxValue if not np.isnan(secondMaxValue) else df.max().max()

        ax = sns.heatmap(df.T, cmap="RdBu", annot=showValues, fmt=".2f", linewidths=.5, vmin=vmin, vmax=vmax, square=False, ax=ax)
        ax.invert_xaxis()
        ax.set_xlabel("")

        # dynamic x-axis and y-axis tick labels based on DataFrame length
        ax.set_yticks(np.arange(0.5, len(df.columns), 1))
        ax.set_yticklabels(range(len(df.columns), 0, -1))
        ax.set_xticks(np.arange(0.5, len(df), 1))
        ax.set_xticklabels(range(len(df), 0, -1))

        ax.set_title(f'Vision Heatmap - {getHeightFromFile(csvFile)}m {dfAverage}')#! Round to 3 nums after dot if 2 long

    # prevent overlapping
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    directoryPath = "C:\\Users\\rakra\\Common\\src\\main\\java\\frc\\robot\\vision\\heatMap\\heatMapsCSVs"
    plotHeatmaps(directoryPath)