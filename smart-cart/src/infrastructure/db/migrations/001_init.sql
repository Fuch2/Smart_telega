-- ===== src/infrastructure/db/migrations/001_init.sql =====
-- Исправлено:
--   - таблица переименована slots → slot_states (соответствует репозиториям)
--   - убрана таблица slots (дублировала slot_states с другой схемой)
--   - добавлены все таблицы, которые используются в коде

PRAGMA foreign_keys = ON;

-- =========================================================
-- modules
-- =========================================================
CREATE TABLE IF NOT EXISTS modules (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    serial       TEXT    NOT NULL UNIQUE COLLATE NOCASE,
    slot_count   INTEGER NOT NULL DEFAULT 24 CHECK (slot_count > 0),
    firmware     TEXT    NOT NULL DEFAULT '',
    status       TEXT    NOT NULL DEFAULT 'OFFLINE'
                         CHECK (status IN ('ONLINE','OFFLINE','MAINT')),
    created_at   DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at   DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX IF NOT EXISTS idx_modules_status
    ON modules(status);

-- =========================================================
-- slot_states  (используется ReelRepositorySqlite + ModuleRepositorySqlite)
-- =========================================================
CREATE TABLE IF NOT EXISTS slot_states (
    module_id   INTEGER NOT NULL,
    slot_index  INTEGER NOT NULL CHECK (slot_index >= 1),
    state       TEXT    NOT NULL DEFAULT 'FREE'
                        CHECK (state IN ('FREE','OCCUPIED','RESERVED','ERROR')),
    updated_at  DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    PRIMARY KEY (module_id, slot_index),
    FOREIGN KEY (module_id) REFERENCES modules(id)
        ON DELETE CASCADE ON UPDATE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_slot_states_module
    ON slot_states(module_id, state);

-- =========================================================
-- reels
-- =========================================================
CREATE TABLE IF NOT EXISTS reels (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    barcode     TEXT    NOT NULL,
    module_id   INTEGER NOT NULL,
    slot_index  INTEGER NOT NULL,
    placed_at   DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    removed_at  DATETIME,

    FOREIGN KEY (module_id) REFERENCES modules(id)
        ON DELETE CASCADE ON UPDATE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_reels_module_slot
    ON reels(module_id, slot_index);

CREATE INDEX IF NOT EXISTS idx_reels_barcode
    ON reels(barcode);

CREATE INDEX IF NOT EXISTS idx_reels_active
    ON reels(module_id, slot_index, removed_at)
    WHERE removed_at IS NULL;

-- =========================================================
-- operations
-- =========================================================
CREATE TABLE IF NOT EXISTS operations (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    type        TEXT    NOT NULL
                        CHECK (type IN ('ADD_REEL','REMOVE_REEL','REPLACE_REEL')),
    status      TEXT    NOT NULL DEFAULT 'IN_PROGRESS'
                        CHECK (status IN ('IN_PROGRESS','COMPLETED','CANCELLED','FAILED')),
    module_id   INTEGER NOT NULL,
    slot_index  INTEGER NOT NULL,
    barcode     TEXT    NOT NULL DEFAULT '',
    started_at  DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    finished_at DATETIME,

    FOREIGN KEY (module_id) REFERENCES modules(id)
        ON DELETE CASCADE ON UPDATE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_operations_unfinished
    ON operations(status, started_at)
    WHERE status = 'IN_PROGRESS';

-- =========================================================
-- event_log
-- =========================================================
CREATE TABLE IF NOT EXISTS event_log (
    id      INTEGER PRIMARY KEY AUTOINCREMENT,
    ts      DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    level   TEXT     NOT NULL,
    code    TEXT     NOT NULL,
    message TEXT     NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_event_log_ts
    ON event_log(ts DESC);

-- =========================================================
-- schema_migrations  (управляется SqliteConnection::runMigrations)
-- =========================================================
CREATE TABLE IF NOT EXISTS schema_migrations (
    filename   TEXT PRIMARY KEY NOT NULL,
    applied_at TEXT NOT NULL DEFAULT (datetime('now'))
);
