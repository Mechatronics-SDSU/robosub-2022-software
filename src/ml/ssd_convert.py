"""Convert vott csv output to ssd mobilenet csv.
Adds in dummy data for latter parameters.
"""
import shutil
import pandas as pd
import sys
import os
import random as rng

LABEL_NAMES = 'class-descriptions-boxable.csv'
LABEL_SOURCE = 'xclick'
LABEL_DEFAULT_CONFIDENCE = 1


def convert_vott_csv_ssd(path_to_csv: str, image_data_loc: str) -> None:
    """Assume input to the function is already randomized for naming using another script.
    """
    print('Running...')
    # Test for labels output
    try:
        f = open(LABEL_NAMES, 'r+')
    except FileNotFoundError:
        print(f'[SSD CONVERT] {LABEL_NAMES} does not exist, creating it...')
        f_1 = open(LABEL_NAMES, 'x')
        f_1.close()
        f = open(LABEL_NAMES, 'r+')
    parsed_label_lines = [i.rstrip('\n') for i in f.readlines()]
    parsed_label_lines_out = []
    label_strings_ds = []
    for i in range(len(parsed_label_lines)):
        parsed_label_lines_out.append(parsed_label_lines[i].split(','))
        label_strings_ds.append(parsed_label_lines_out[i][1])
    # print(parsed_label_lines_out)
    # print(label_strings_ds)
    f.close()
    # Load vott csv
    try:
        df = pd.read_csv(path_to_csv)
    except OSError:
        print('OSError, cannot find file specifiied.')
        sys.exit(2)
    # print(df.head())
    df['old_labels'] = df['label']
    print(df)
    # Parse labels, generate new hex for each
    for i in range(int(df.size / 7)):
        label = df.at[i, 'label']
        if label in label_strings_ds:
            ind = label_strings_ds.index(label)
            df.at[i, 'label'] = parsed_label_lines_out[ind][0]
        else:
            enumeration = len(label_strings_ds)
            enumeration_string = f'/m/{f"{enumeration}".zfill(5)}'
            parsed_label_lines_out.append([enumeration_string, df.at[i, 'label']])
            label_strings_ds.append(df.at[i, 'label'])
            df.at[i, 'label'] = enumeration_string
    # Write out the new labels from the vott csv, if any
    f = open(LABEL_NAMES, 'w+')
    for i in parsed_label_lines_out:
        f.write(f'{i[0]},{i[1]}\n')
    # Parse image names, remove file extension
    for i in range(int(df.size / 7)):
        image_loc = df.at[i, 'image']
        df.at[i, 'image'] = image_loc.split('.')[0]
    f.close()
    # Rearrange dataframe
    df['ImageID'] = df['image']
    df['Source'] = LABEL_SOURCE
    df['LabelName'] = df['label']
    df['Confidence'] = LABEL_DEFAULT_CONFIDENCE
    df['XMin'] = df['xmin']
    df['XMax'] = df['xmax']
    df['YMin'] = df['ymin']
    df['YMax'] = df['ymax']
    df['IsOccluded'] = 0
    df['IsTruncated'] = 0
    df['IsGroupOf'] = 0
    df['IsDepiction'] = 0
    df['IsInside'] = 0
    df['id'] = df['label']
    df['ClassName'] = df['old_labels']
    df = df.drop(['image', 'xmin', 'ymin', 'xmax', 'ymax', 'label', 'old_labels'], axis=1)
    df.to_csv('out.csv', index=False)
    # Train test validation split
    df.drop(df.index[:], axis=0, inplace=True)
    df_train = df.copy(deep=True)
    df_validation = df.copy(deep=True)
    df_test = df.copy(deep=True)
    dfs = [df_train, df_validation, df_test]
    f = open('out.csv', 'r+')
    labels = [i.rstrip('\n') for i in f.readlines()]
    labels = labels[1:]
    f.close()
    # Split data into train/test/valid, 92/6/2 ratio
    for i in range(len(labels)):
        parsed_entry = labels[i].split(',')
        split = rng.randint(0, 99)
        if split < 93:  # Train
            ind = 0
        elif (split >= 93) and (split < 95):  # Validation
            ind = 1
        else:  # Test
            ind = 2
        dfs[ind] = pd.concat(
            [dfs[ind], pd.DataFrame([parsed_entry], columns=['ImageID', 'Source', 'LabelName', 'Confidence',
                                                             'XMin', 'XMax', 'YMin', 'YMax', 'IsOccluded',
                                                             'IsTruncated', 'IsGroupOf', 'IsDepiction',
                                                             'IsInside', 'id', 'ClassName'])],
            ignore_index=True)
    # Debug
    print(dfs[0])
    print(dfs[1])
    print(dfs[2])
    # Write out dataframes
    dfs[0].to_csv('train-annotations-bbox.csv', index=False)
    dfs[1].to_csv('validation-annotations-bbox.csv', index=False)
    dfs[2].to_csv('test-annotations-bbox.csv', index=False)
    # Sort images
    train_dir = '/../train'
    valid_dir = '/../validation'
    test_dir = '/../test'
    dirs = [train_dir, valid_dir, test_dir]
    os.mkdir(image_data_loc + train_dir)
    os.mkdir(image_data_loc + valid_dir)
    os.mkdir(image_data_loc + test_dir)
    loaded_imgs = []
    print('Copying image data...')
    for n in range(len(dfs)):
        for i in range(int(dfs[n].size / 15)):
            fs_name = str(dfs[n].at[i, 'ImageID']) + '.jpg'
            if fs_name not in loaded_imgs:
                shutil.copy(image_data_loc + f'/{fs_name}', f'{image_data_loc}{dirs[n]}/{fs_name}')
                loaded_imgs.append(fs_name)


if __name__ == '__main__':
    if len(sys.argv) > 2:
        # images = sys.argv[1].strip(' ')
        csv = sys.argv[1].strip(' ')
        images = sys.argv[2].strip(' ')
        convert_vott_csv_ssd(path_to_csv=csv, image_data_loc=images)
    else:
        print('Error, argv < 3. (Did you pass the vott csv location and the image location to this program?)')
        sys.exit(1)
