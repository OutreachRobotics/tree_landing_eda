from statsmodels.stats.outliers_influence import variance_inflation_factor
from statsmodels.tools.tools import add_constant

import config
import matplotlib.pyplot as plt
import os
import pandas as pd
import seaborn as sns

def analyze_multicollinearity(_ignore_list: list[str] = [], _specie: str = ''):
    """
    Loads data from a CSV and computes multicollinearity diagnostics.

    This function performs two main checks:
    1. A correlation matrix heatmap for a quick visual inspection.
    2. A Variance Inflation Factor (VIF) analysis for a robust statistical measure.

    Args:
        filepath (str): The path to the input CSV file.
    """
    try:
        df = pd.read_csv(os.path.join(config.OUTPUTS_PATH, config.OUTPUT_CSV))
        if _specie is not '':
            mask = df['specie'] == _specie
            df = df[mask]
    except FileNotFoundError:
        print(f"Error: The file was not found at '{os.path.join(config.OUTPUTS_PATH, config.OUTPUT_CSV)}'")
        return

    # --- 1. Correlation Matrix ---
    print("--- 1. Correlation Matrix Analysis ---")
    
    # Select only numeric columns for correlation calculation
    ignore_list = config.IGNORE_LIST.copy()
    ignore_list.extend(['success','specie'])
    ignore_list.extend(_ignore_list)

    cols_to_drop = []
    for pattern in ignore_list:
        # filter(like=...) finds all columns containing the pattern string
        matching_cols = df.filter(like=pattern).columns
        cols_to_drop.extend(matching_cols)
    cols_to_drop = list(set(cols_to_drop))

    df_to_analyze = df.dropna().drop(columns=cols_to_drop, errors='ignore')

    corr_matrix = df_to_analyze.corr()

    # It's often helpful to look at the raw matrix first
    print("Correlation Matrix:")
    print(corr_matrix)

    # Now, let's visualize it with a heatmap for better interpretation
    plt.figure(figsize=(12, 10))
    sns.heatmap(corr_matrix, annot=True, cmap='coolwarm', fmt=".2f")
    plt.title('Correlation Matrix Heatmap')
    plt.show()

    print("\nInterpretation:")
    print("Look for values close to +1.0 or -1.0 (excluding the main diagonal).")
    print("A common rule of thumb is that a correlation > 0.8 or < -0.8 indicates high collinearity.")
    
    # --- 2. Variance Inflation Factor (VIF) ---
    print("\n--- 2. Variance Inflation Factor (VIF) Analysis ---")
    
    # VIF is a more rigorous test. It requires a constant (intercept) term.
    # We use the same numeric columns as before.
    X = add_constant(df_to_analyze)

    # Create a DataFrame to hold the VIF results
    vif_df = pd.DataFrame()
    vif_df["Variable"] = X.columns
    vif_df["VIF"] = [variance_inflation_factor(X.values, i) for i in range(X.shape[1])]

    print("Variance Inflation Factor (VIF):")
    # The VIF for the constant is usually very high and not interpretable, so we can drop it.
    print(vif_df[vif_df['Variable'] != 'const'])
    
    print("\nInterpretation:")
    print("VIF = 1: No correlation")
    print("1 < VIF < 5: Moderate correlation")
    print("VIF > 5 or 10: High correlation (this threshold is a rule of thumb)")
    print("A VIF of infinity (inf) means perfect collinearity (e.g., one variable is derived from another).")

def plot_pair_plot(_ignore_list: list[str] = [], _specie: str = ''):
    """
    Generates and displays a pair plot (scatter plot matrix) for numeric variables.
    
    This is a great way to visually explore the data before diving into the numbers.
    A perfect task for a relaxing Monday evening.
    """
    try:
        df = pd.read_csv(os.path.join(config.OUTPUTS_PATH, config.OUTPUT_CSV))
        if _specie is not '':
            mask = df['specie'] == _specie
            df = df[mask]
    except FileNotFoundError:
        print(f"Error: The file was not found at '{os.path.join(config.OUTPUTS_PATH, config.OUTPUT_CSV)}'")
        return

    print("\n--- 2. Generating Pair Plot ---")
    ignore_list = config.IGNORE_LIST.copy()
    ignore_list.extend(['specie'])
    ignore_list.extend(_ignore_list)

    cols_to_drop = []
    for pattern in ignore_list:
        # filter(like=...) finds all columns containing the pattern string
        matching_cols = df.filter(like=pattern).columns
        cols_to_drop.extend(matching_cols)
    cols_to_drop = list(set(cols_to_drop))

    df_to_plot = df.dropna().drop(columns=cols_to_drop)

    if df_to_plot.shape[1] < 2:
        print("Not enough numeric columns to generate a pair plot.")
        return
        
    # A practical check: Pair plots with too many variables can be slow and unreadable.
    if df_to_plot.shape[1] > 10:
        print(f"Warning: Generating a pair plot for {df_to_plot.shape[1]} variables."
              "This may take some time and be difficult to read.")
        
    cols_to_plot = [
        col for col in df_to_plot
    ]

    if len(cols_to_plot) < 1:
        print("Not enough numeric columns to generate a pair plot.")
        return
    
    outcome_colors = {
        1.0: 'green',
        0.0: 'red'
    }

    hue_column = 'success'

    if hue_column in cols_to_plot:
        cols_to_plot.remove(hue_column)

    g = sns.pairplot(
            df_to_plot,
            vars=cols_to_plot,
            hue=hue_column,
            palette=outcome_colors,
            diag_kind='kde'
        )
    
    # 1. Iterate through all the axes in the pairplot grid
    for ax in g.axes.flatten():
        # Check if the axis is not None (for empty spaces in the grid)
        if ax is not None:
            # 2. Rotate the x-axis labels
            ax.set_xlabel(ax.get_xlabel(), rotation=45, horizontalalignment='right')
            
            # 3. Rotate the y-axis labels
            ax.set_ylabel(ax.get_ylabel(), rotation=45, horizontalalignment='right')
        
    g.fig.suptitle(f'Pair Plot Colored by "{hue_column}"', y=1.02)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    specie = 'red_maple'
    ignore_list = [
        'Min_Curvature','Mean_Curvature','Gaussian_Curvature',
        'Distance_Top','Distance_Tree_Center_2D','Ratio_Tree_Center_2D','Distance_Tree_Center_2D',
        'Distance_Tree_Highest_Point_2D','Ratio_Tree_Highest_Point_2D',
        'Distance_Tree_Center_3D','Distance_Tree_Highest_Point_3D',
        'Tree_Minor_Diameter'
    ]

    analyze_multicollinearity(ignore_list)
    plot_pair_plot(ignore_list)
        
    # analyze_multicollinearity(ignore_list, specie)
    # plot_pair_plot(ignore_list, specie)
