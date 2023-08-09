import sqlite3
import numpy as np

db_path = '/home/hoangqc/COLMAP/test-ZED-home/home.db'
traj_path = '/home/hoangqc/COLMAP/test-ZED-home/pose_left.txt'

poses = np.loadtxt(traj_path)
poses = poses[::30]
xyz = poses[:, 0:3]

#open SQLite database
conn = sqlite3.connect(db_path)

#get all tables
cursor = conn.cursor()
cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
tables = cursor.fetchall()
print(tables)

#get data from 'images' table
cursor.execute("SELECT * FROM images")
images = cursor.fetchall()
print(images)

# modify tx ty tz in 'images' table
for i in range(len(images)):
    cursor.execute("UPDATE images SET prior_tx = ?, prior_ty = ?, prior_tz = ? WHERE image_id = ?", (xyz[i,0], xyz[i,1], xyz[i,2], i+1))

# save changes
conn.commit()
conn.close()