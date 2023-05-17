import csv
import time

csv_path = "data/csv/" + time.strftime("%Y%m%d_%H%M%S") + ".csv"

with open(csv_path, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["timestamp", "undercup", "whilist"])
    writer.writerow([time.time(), [0,0,0,0,0,0], [0,0,0,0,0,0]])
    time.sleep(199)
    writer.writerow([time.time(), [0,0,0,0,0,0], [0,0,0,0,0,0]])
    writer.writerow([time.time(), [0,0,0,0,0,0], [0,0,0,0,0,0]])