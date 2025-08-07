from sklearn.preprocessing import LabelEncoder
from sklearn import tree

import config
import dtreeviz
import pandas as pd
import os

def decision_tree(_ignore_list: list[str] = [], _depth: int = 3):
    output_csv = pd.read_csv(os.path.join(config.OUTPUTS_PATH, config.OUTPUT_CSV))
    raw_len = len(output_csv)

    output_csv = output_csv.dropna()
    # output_csv = output_csv.dropna().reset_index(drop=True)

    if output_csv.empty:
        print('No data remaining after dropping NaN rows. Aborting decision tree.')
        return
    
    ignore_list = config.IGNORE_LIST.copy()
    ignore_list.extend(['success','specie'])
    if _ignore_list:
        ignore_list.extend(_ignore_list)

    le = LabelEncoder()
    output_csv['specie_encoded'] = le.fit_transform(output_csv['specie'])

    dropped_len = raw_len - len(output_csv)
    print('Dropped ' + str(dropped_len) + 'rows containing nan value.')
    print(output_csv)

    output_csv['success'] = (output_csv['success']).astype(int)

    y = output_csv['success']
    X = output_csv.drop(ignore_list, axis=1)
    clf = tree.DecisionTreeClassifier(max_depth=_depth)
    clf = clf.fit(X.values, y.values)

    viz_model = dtreeviz.model(clf,
        X_train=X, y_train=y,
        feature_names=X.columns.tolist(),
        target_name='success',
        class_names=list(clf.classes_),
    )

    v = viz_model.view(fontname='DejaVu Sans')
    v.save(os.path.join(config.OUTPUTS_PATH, config.DECISIONTREE_SVG))
    print(f"Saved decision tree to {os.path.join(config.OUTPUTS_PATH, config.DECISIONTREE_SVG)}")

    importances = clf.feature_importances_
    feature_importance_df = pd.DataFrame({'feature': X.columns, 'importance': importances})
    print(feature_importance_df.sort_values(by='importance', ascending=False))


def main():
    ignore_list = [
        'Curvature_PC1','Curvature_PC2','Gaussian_Curvature',
        'Distance_Top','Distance_Tree_Center_2D','Distance_Tree_Highest_Point_2D',
        'Distance_Tree_Center_3D','Distance_Tree_Highest_Point_3D',
    ]
    ignore_list.extend(['Density','Mean_Curvature','Standard_Deviation','Slope','Tree_Major_Diameter'])
    # ignore_list.extend(['Density'])
    decision_tree(ignore_list, 4)

if __name__=='__main__':
    main()
