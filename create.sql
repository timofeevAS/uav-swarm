-- Init SQLLite3 table for swarm_state;
CREATE TABLE swarm_state (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    iteration INTEGER,
    position_x REAL,
    position_y REAL,
    position_z REAL,
    speed_x REAL,
    speed_y REAL,
    speed_z REAL,
    state TEXT DEFAULT "",
    experiment_id INTEGER,
    target_x REAL,
    target_y REAL,
    target_z REAL,
)
