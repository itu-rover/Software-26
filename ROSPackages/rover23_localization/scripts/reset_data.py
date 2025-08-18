import csv

# Define the filenames of your input and output CSV files
odom_file = "/home/iturover/iturover23_ws/src/rover23_localization/data/odom.csv"
yaw_file = "/home/iturover/iturover23_ws/src/rover23_localization/data/yaw_diff.csv"
liorf_yaw_file = "/home/iturover/iturover23_ws/src/rover23_localization/data/liorf_yaw_diff.csv"

# Function to write data to a CSV file
def write_to_csv(filename, data):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)

data1 = [["0 0"], ["0 0"], ["0 0"]]
data2 = [[0], [0], [0]]

# Write the data to the output files
write_to_csv(odom_file, data1)
write_to_csv(liorf_yaw_file, data2)




