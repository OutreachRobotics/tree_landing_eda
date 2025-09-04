from sklearn import tree

import config
import dtreeviz
import pandas as pd
import os

def extract_csv(_filepath):
    output_csv = pd.read_csv(_filepath)
    raw_len = len(output_csv)

    output_csv = output_csv.dropna()
    output_csv = output_csv.sample(frac=1).reset_index(drop=True) # Shuffle the data

    dropped_len = raw_len - len(output_csv)
    print(f'Dropped {dropped_len} rows containing nan value, {len(output_csv)} rows left.')
    print(output_csv)

    if output_csv.empty:
        print('No data remaining after dropping NaN rows. Aborting decision tree.')
        return
    
    output_csv['success'] = (output_csv['success']).astype(int)

    return output_csv

def print_tree_info(_clf, _X):
    importances = _clf.feature_importances_
    feature_importance_df = pd.DataFrame({'feature': _X.columns, 'importance': importances})
    print()
    print(feature_importance_df.sort_values(by='importance', ascending=False))

    tree_ = _clf.tree_
    feature_names = _X.columns.tolist()
    print()
    print("--- Extracted Decision Tree Thresholds ---")

    def recurse(node_id, depth):
        # A node is a "split" node if its children are different
        if tree_.children_left[node_id] != tree_.children_right[node_id]:
            # Get the feature and threshold for the current split
            feature = feature_names[tree_.feature[node_id]]
            threshold = tree_.threshold[node_id]
            
            # Print the formatted output including the depth
            print(
                f"Level {depth}, Split Node {node_id}: "
                f"if {feature} <= {threshold:.8f}"
            )

            # Recurse on the left and right children, incrementing the depth
            recurse(tree_.children_left[node_id], depth + 1)
            recurse(tree_.children_right[node_id], depth + 1)

    recurse(0, 0)

def tree_viz(_clf, _X, _y, _output_name):
    viz_model = dtreeviz.model(_clf,
        X_train=_X, y_train=_y,
        feature_names=_X.columns.tolist(),
        target_name=_y.name,
        class_names=list(_clf.classes_),
    )

    v = viz_model.view(fontname='DejaVu Sans')
    v.save(os.path.join(config.OUTPUTS_PATH, _output_name))
    print(f"Saved decision tree to {os.path.join(config.OUTPUTS_PATH, _output_name)}")


def decision_tree(_depth: int = 3, _ignore_list: list[str] = [], _specie: str = ''):
    landings_df = extract_csv(os.path.join(config.OUTPUTS_PATH, config.OUTPUT_CSV))

    filepath = config.DECISIONTREE_SVG
    if _specie is not '':
        filepath = f"{_specie}_{config.DECISIONTREE_SVG}"
        mask = landings_df['specie'] == _specie
        landings_df = landings_df[mask]

    landings_df = pd.get_dummies(landings_df, columns=['specie'], prefix='specie') # REPLACE with One-Hot Encoding using pd.get_dummies()
    
    ignore_list = config.IGNORE_LIST.copy()
    ignore_list.extend(['success'])
    if _ignore_list:
        ignore_list.extend(_ignore_list)

    cols_to_drop = []
    for pattern in ignore_list:
        # filter(like=...) finds all columns containing the pattern string
        matching_cols = landings_df.filter(like=pattern).columns
        cols_to_drop.extend(matching_cols)
    cols_to_drop = list(set(cols_to_drop))

    y = landings_df['success']
    X = landings_df.drop(cols_to_drop, axis=1)
    clf = tree.DecisionTreeClassifier(max_depth=_depth)
    clf = clf.fit(X.values, y.values)

    tree_viz(clf, X, y, filepath)
    print_tree_info(clf, X)

def decision_tree_by_scale(_depth: int = 3, _ignore_list: list[str] = [], _scale: float = 0.0, _specie: str = ''):
    landings_df = extract_csv(os.path.join(config.OUTPUTS_PATH, config.OUTPUT_CSV))

    scale_suffix = f"_scale_{_scale:.1f}"
    cols_to_keep = [
        col for col in landings_df.columns
        if '_scale_' not in col or scale_suffix in col
    ]
    landings_df = landings_df[cols_to_keep]

    base_name, extension = os.path.splitext(config.DECISIONTREE_SVG) # e.g., ('decision_tree', '.svg')
    filename_parts = [base_name]
    if _specie:
        filename_parts.append(_specie)
    filename_parts.append("scale")
    filename_parts.append(f"{_scale:.1f}") # Add the formatted scale
    filepath = "_".join(filename_parts) + extension
    print(f"Outputting SVG to: {filepath}")

    if _specie is not '':
        mask = landings_df['specie'] == _specie
        landings_df = landings_df[mask]

    landings_df = pd.get_dummies(landings_df, columns=['specie'], prefix='specie') # REPLACE with One-Hot Encoding using pd.get_dummies()
    
    ignore_list = config.IGNORE_LIST.copy()
    ignore_list.extend(['success'])
    if _ignore_list:
        ignore_list.extend(_ignore_list)

    cols_to_drop = []
    for pattern in ignore_list:
        # filter(like=...) finds all columns containing the pattern string
        matching_cols = landings_df.filter(like=pattern).columns
        cols_to_drop.extend(matching_cols)
    cols_to_drop = list(set(cols_to_drop))

    y = landings_df['success']
    X = landings_df.drop(cols_to_drop, axis=1)
    clf = tree.DecisionTreeClassifier(max_depth=_depth)
    clf = clf.fit(X.values, y.values)

    tree_viz(clf, X, y, filepath)
    print_tree_info(clf, X)

def decision_tree_by_tree(_depth: int = 3, _specie: str = ''):
    landings_df = extract_csv(os.path.join(config.OUTPUTS_PATH, config.OUTPUT_CSV))

    filepath = config.DECISIONTREE_AGGREGATE_SVG
    if _specie is not '':
        filepath = f"{_specie}_{config.DECISIONTREE_AGGREGATE_SVG}"
        mask = landings_df['specie'] == _specie
        landings_df = landings_df[mask]

    # --- AGGREGATION STEP ---
    # 1. Group by the tree index ('idx')
    # 2. For each tree, calculate the success rate (mean of the 'success' column)
    # 3. For tree-specific features, take the first value (since they are all the same for that tree)
    tree_df = landings_df.groupby('idx').agg(
        success_rate=('success', 'mean'),
        specie=('specie', 'first'),
        Tree_Major_Diameter=('Tree_Major_Diameter', 'mean'),
        Tree_Minor_Diameter=('Tree_Minor_Diameter', 'mean')
    ).reset_index()

    print("--- Aggregated Data (One Row Per Tree) ---")
    print(tree_df.head())

    # --- PREPARE DATA FOR THE DECISION TREE ---
    # The new target is whether a tree is "good" to land on.
    # Let's define a "good" tree as one with a success rate of 50% or more.
    tree_df['is_good_tree'] = (tree_df['success_rate'] >= 0.5).astype(int)
    
    # One-Hot Encode the 'specie' column
    tree_df = pd.get_dummies(tree_df, columns=['specie'], prefix='specie')

    # Define features (X) and target (y)
    y = tree_df['is_good_tree']
    # We drop the original identifiers and the success rate itself
    X = tree_df.drop(columns=['idx', 'success_rate', 'is_good_tree'])

    # --- TRAIN THE DECISION TREE ---
    clf = tree.DecisionTreeClassifier(max_depth=_depth, random_state=42)
    clf.fit(X, y)

    tree_viz(clf, X, y, filepath)
    print_tree_info(clf, X)

def get_unique_scales(df):
    """
    Extracts unique scale values from a DataFrame's column names.
    
    Args:
        df (pd.DataFrame): The DataFrame with wide-format data.
        
    Returns:
        list[float]: A sorted list of the unique scales found.
    """
    scales = set()  # Use a set to automatically handle duplicates
    for col in df.columns:
        if '_scale_' in col:
            try:
                # Split 'Density_scale_1.0' -> ['Density', '1.0']
                scale_str = col.split('_scale_')[-1]
                # Convert the string '1.0' to the float 1.0 and add to the set
                scales.add(float(scale_str))
            except (ValueError, IndexError):
                # This safely ignores columns that might have '_scale_'
                # but aren't followed by a valid number.
                print(f"Warning: Could not parse scale from column: {col}")
    
    # Convert the set to a list and sort it for a predictable order
    return sorted(list(scales))

def generate_decision_trees(_ignore_list: list[str] = [], _specie: str = '', _scale: float = 0.0):
    decision_tree(3, _ignore_list)
    decision_tree_by_tree(3)

    if _specie == '':
        landings_df = extract_csv(os.path.join(config.OUTPUTS_PATH, config.OUTPUT_CSV))
        species = landings_df['specie'].unique()

        for specie in species:
            decision_tree(3, _ignore_list, specie)
            decision_tree_by_tree(2, specie)
    else:
        decision_tree(3, _ignore_list, _specie)
        decision_tree_by_tree(2, _specie)

        landings_df = extract_csv(os.path.join(config.OUTPUTS_PATH, config.OUTPUT_CSV))
        scales = get_unique_scales(landings_df)

        for scale in scales:
            decision_tree_by_scale(3, _ignore_list, scale, _specie)

    if _scale == 0.0:
        landings_df = extract_csv(os.path.join(config.OUTPUTS_PATH, config.OUTPUT_CSV))
        scales = get_unique_scales(landings_df)

        for scale in scales:
            decision_tree_by_scale(3, _ignore_list, scale)
    else:
        decision_tree_by_scale(3, _ignore_list, _scale)

def main():
    specie = 'red_maple'
    ignore_list = config.IGNORE_LIST.copy()
    # ignore_list.extend(['Mean_Curvature','Gaussian_Curvature'])
    ignore_list.extend(['Distance_Top','Distance_Tree_Center_2D','Distance_Tree_Center_3D',
                        'Distance_Tree_Highest_Point_2D','Distance_Tree_Highest_Point_3D',
                        'scale_0.5','scale_3.0','Slope','Gaussian_Curvature','Min_Curvature'])

    # decision_tree(3, ignore_list)
    # decision_tree_by_tree(3)

    # decision_tree(3, ignore_list, specie)
    # decision_tree_by_tree(3, specie)

    generate_decision_trees(ignore_list)
    generate_decision_trees(ignore_list, 'sugar_maple')

if __name__=='__main__':
    main()
