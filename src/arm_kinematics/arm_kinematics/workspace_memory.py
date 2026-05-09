import numpy as np
import os

BASE_DIR = os.path.dirname(__file__)
DB_PATH = os.path.join(BASE_DIR, "../data/workspace_db.npy")

def load_db():
    if os.path.exists(DB_PATH):
        return np.load(DB_PATH)
    return np.empty((0, 9))  # x,y,z + 6 joints

def save_db(data):
    np.save(DB_PATH, data)

def add_entry(db, point, angles):
    new_row = np.array(list(point) + list(angles))
    db = np.vstack([db, new_row])
    save_db(db)
    return db