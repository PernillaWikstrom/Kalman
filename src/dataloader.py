import numpy as np
import pandas as pd


def load_data(path2path_to_measurements, path_to_ground_truth_data, dt):
    df_measurement = pd.read_csv(path2path_to_measurements, sep=",")
    df_measurement["time"] = np.arange(0, len(df_measurement), dt)
    df_ground_truth = pd.read_csv(path_to_ground_truth_data, sep=",")
    return df_measurement, df_ground_truth
