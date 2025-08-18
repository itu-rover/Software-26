#!/usr/bin/env python3

import numpy as np

# Load collected data
magnetometer_data = np.load('/home/iturover/iturover23_ws/src/rover23_localization/scripts/magnetometer_data.npy')

# Compute offsets
mag_x = magnetometer_data[:, 0]
mag_y = magnetometer_data[:, 1]
offset_x = np.mean(mag_x)
offset_y = np.mean(mag_y)

# Compute scale factors
scale_x = (np.max(mag_x) - np.min(mag_x)) / 2
scale_y = (np.max(mag_y) - np.min(mag_y)) / 2
scale_factor = (scale_x + scale_y) / 2
scale_x /= scale_factor
scale_y /= scale_factor

# Save calibration parameters
calibration_params = {'offset_x': offset_x, 'offset_y': offset_y, 'scale_x': scale_x, 'scale_y': scale_y}
np.save('/home/iturover/iturover23_ws/src/rover23_localization/scripts/calibration_params.npy', calibration_params)
