PRAGMA foreign_keys = ON;

-- =========================================================
-- modules
-- serial + slot_count + firmware + status — согласовано с ModuleInfo
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
-- slots
-- slot_index 1..N, led_index — физический LED на STM32
-- =========================================================
CREATE TABLE IF NOT EXISTS slots (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    module_id   INTEGER NOT NULL,
    slot_index  INTEGER NOT NULL CHECK (slot_index >= 1),
    led_index   INTEGER NOT NULL DEFAULT 0,
    state       TEXT    NOT NULL DEFAULT 'FREE'
                        CHECK (state IN ('FREE','OCCUPIED','RESERVED','ERROR')),
    updated_at  DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (module_id) REFERENCES modules(id)
        ON DELETE CASCADE ON UPDATE CASCADE,
    UNIQUE (module_id, slot_index)
);

CREATE INDEX IF NOT EXISTS idx_slots_module_state
    ON slots(module_id, state);

-- =========================================================
-- reels
-- barcode — штрихкод катушки, placed_at/removed_at — жизненный цикл
-- =========================================================
CREATE TABLE IF NOT EXISTS reels (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    barcode     TEXT    NOT NULL,
    module_id   INTEGER NOT NULL,
    slot_index  INTEGER NOT NULL,
    placed_at   DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    removed_at  DATETIME,           -- NULL = катушка ещё в слоте

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
-- тип: ADD_REEL / REMOVE_REEL / REPLACE_REEL
-- статус: IN_PROGRESS / COMPLETED / CANCELLED / FAILED
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
    finished_at DATETIME,           -- NULL = ещё выполняется

    FOREIGN KEY (module_id) REFERENCES modules(id)
        ON DELETE CASCADE ON UPDATE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_operations_unfinished
    ON operations(status, started_at)
    WHERE status = 'IN_PROGRESS';

-- =========================================================
-- event_log  (IEventLogger → SqliteEventLogger)
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
