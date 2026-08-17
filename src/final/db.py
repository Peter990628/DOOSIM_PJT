from __future__ import annotations

import os
import sqlite3
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path

DEFAULT_DB_PATH = Path(__file__).resolve().parent / "hospital_gui.db"

PATIENTS = {
    1: {"name": "박인천", "birth_date": "1960-07-23"},
    2: {"name": "김서울", "birth_date": "2000-11-02"},
    3: {"name": "서수원", "birth_date": "1990-02-10"},
}


def db_path():
    configured = os.getenv("GUI_DB_PATH")
    return Path(configured).expanduser().resolve() if configured else DEFAULT_DB_PATH


def now_text():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


@contextmanager
def connect():
    con = sqlite3.connect(db_path(), timeout=5)
    con.row_factory = sqlite3.Row
    try:
        yield con
        con.commit()
    except Exception:
        con.rollback()
        raise
    finally:
        con.close()


def _ensure_column(con, table, name, ddl):
    columns = {item["name"] for item in con.execute(f"PRAGMA table_info({table})").fetchall()}
    if name not in columns:
        con.execute(f"ALTER TABLE {table} ADD COLUMN {name} {ddl}")


def init_db(*, reset=False):
    """Create the DB schema and optionally reset all runtime/demo state.

    app.py calls this with reset=True so every server process starts from the
    same clean GUI state, regardless of what was stored in the previous run.
    """
    with connect() as con:
        con.executescript(
            """
            CREATE TABLE IF NOT EXISTS amrs (
                name TEXT PRIMARY KEY,
                floor INTEGER NOT NULL,
                room TEXT NOT NULL,
                status TEXT NOT NULL,
                x REAL,
                y REAL,
                yaw REAL,
                updated_at TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS beds (
                id INTEGER PRIMARY KEY,
                label TEXT NOT NULL,
                patient_name TEXT,
                birth_date TEXT,
                floor INTEGER NOT NULL,
                room TEXT NOT NULL,
                status TEXT NOT NULL,
                assigned_amr TEXT,
                x REAL,
                y REAL,
                yaw REAL,
                updated_at TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS jobs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                kind TEXT NOT NULL,
                amr_name TEXT NOT NULL,
                bed_id INTEGER,
                dest_floor INTEGER NOT NULL,
                dest_room TEXT NOT NULL,
                destination_pending INTEGER NOT NULL DEFAULT 0,
                phase TEXT NOT NULL,
                origin_floor INTEGER,
                origin_room TEXT,
                resume_phase TEXT,
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                level TEXT NOT NULL,
                message TEXT NOT NULL,
                created_at TEXT NOT NULL
            );
            """
        )
        _ensure_column(con, "beds", "patient_name", "TEXT")
        _ensure_column(con, "beds", "birth_date", "TEXT")
        _ensure_column(con, "beds", "x", "REAL")
        _ensure_column(con, "beds", "y", "REAL")
        _ensure_column(con, "beds", "yaw", "REAL")
        _ensure_column(con, "jobs", "destination_pending", "INTEGER NOT NULL DEFAULT 0")
        _ensure_column(con, "jobs", "origin_floor", "INTEGER")
        _ensure_column(con, "jobs", "origin_room", "TEXT")
        _ensure_column(con, "jobs", "resume_phase", "TEXT")

        timestamp = now_text()

        if reset:
            # A GUI restart is a new demo session: remove all previous runtime
            # state and logs, then seed only the requested initial entities.
            con.execute("DELETE FROM jobs")
            con.execute("DELETE FROM events")
            con.execute("DELETE FROM beds")
            con.execute("DELETE FROM amrs")
            con.execute("DELETE FROM sqlite_sequence WHERE name IN ('jobs', 'events')")

        con.executemany(
            "INSERT OR IGNORE INTO amrs(name, floor, room, status, x, y, yaw, updated_at) VALUES(?,?,?,?,?,?,?,?)",
            [
                ("AMR-01", 1, "보관실", "보관실 대기", None, None, None, timestamp),
                ("AMR-02", 1, "보관실", "보관실 대기", None, None, None, timestamp),
            ],
        )
        con.executemany(
            "INSERT OR IGNORE INTO beds(id, label, patient_name, birth_date, floor, room, status, assigned_amr, updated_at) VALUES(?,?,?,?,?,?,?,?,?)",
            [
                (1, "박인천", "박인천", "1960-07-23", 1, "병실", "대기", None, timestamp),
                (2, "김서울", "김서울", "2000-11-02", 1, "병실", "대기", None, timestamp),
                (3, "서수원", "서수원", "1990-02-10", 1, "병실", "대기", None, timestamp),
            ],
        )

        # Keep schema-only calls backward-safe while app startup itself always
        # uses reset=True above.
        con.execute("UPDATE amrs SET room='보관실', status='보관실 대기' WHERE room='충전실' AND status='충전실 대기'")
        for patient_id, info in PATIENTS.items():
            con.execute(
                "UPDATE beds SET label=?, patient_name=?, birth_date=? WHERE id=?",
                (info["name"], info["name"], info["birth_date"], patient_id),
            )

        # AMR/환자 개별 초기 좌표는 사용하지 않습니다. 실제 AMR 위치는 /world_pose 수신 후 갱신됩니다.



def rows(query, params=()):
    with connect() as con:
        return [dict(item) for item in con.execute(query, params).fetchall()]


def row(query, params=()):
    with connect() as con:
        item = con.execute(query, params).fetchone()
        return dict(item) if item else None


def execute(query, params=()):
    with connect() as con:
        cur = con.execute(query, params)
        return cur.lastrowid


def add_event(message: str, level: str = "INFO"):
    execute(
        "INSERT INTO events(level, message, created_at) VALUES(?,?,?)",
        (level, message, now_text()),
    )
