from sklearn.preprocessing import LabelEncoder
from sklearn import tree

import config
import dtreeviz
import pandas as pd
import os

def decision_tree():
    output_csv = pd.read_csv(os.path.join(config.OUTPUTS_PATH, config.OUTPUT_CSV))
    raw_len = len(output_csv)

    output_csv = output_csv.dropna()
    # output_csv = output_csv.dropna().reset_index(drop=True)

    if output_csv.empty:
        print('No data remaining after dropping NaN rows. Aborting decision tree.')
        return

    le = LabelEncoder()
    output_csv['specie_encoded'] = le.fit_transform(output_csv['specie'])

    dropped_len = raw_len - len(output_csv)
    print('Dropped ' + str(dropped_len) + 'rows containing nan value.')
    print(output_csv)

    output_csv['success'] = (output_csv['success']).astype(int)

    y = output_csv['success']
    X = output_csv.drop(['success', 'specie'], axis=1)
    clf = tree.DecisionTreeClassifier(max_depth=3)
    clf = clf.fit(X.values, y.values)

    viz_model = dtreeviz.model(clf,
        X_train=X, y_train=y,
        feature_names=X.columns.tolist(),
        target_name='success',
        class_names=list(clf.classes_),
    )

    v = viz_model.view(fontname='DejaVu Sans')
    v.save(os.path.join(config.OUTPUTS_PATH, config.DECISIONTREE_SVG))


def main():
    decision_tree(16)

if __name__=='__main__':
    main()
