PRAGMA foreign_keys = ON;

-- =========================================================
-- modules: установленный RFID-модуль тележки
-- =========================================================
CREATE TABLE IF NOT EXISTS modules (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    serial_number     TEXT    NOT NULL UNIQUE,
    module_type       TEXT    NOT NULL,
    module_name       TEXT,
    firmware_version  TEXT,
    is_active         INTEGER NOT NULL DEFAULT 1 CHECK (is_active IN (0, 1)),
    installed_at      DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX IF NOT EXISTS idx_modules_active
    ON modules(is_active);

-- =========================================================
-- module_configs: конфигурация модуля (JSON + версия)
-- =========================================================
CREATE TABLE IF NOT EXISTS module_configs (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    module_id         INTEGER NOT NULL,
    config_version    INTEGER NOT NULL DEFAULT 1,
    profile_name      TEXT    NOT NULL,
    slot_count        INTEGER NOT NULL CHECK (slot_count > 0),
    config_json       TEXT    NOT NULL,
    is_active         INTEGER NOT NULL DEFAULT 1 CHECK (is_active IN (0, 1)),
    created_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (module_id) REFERENCES modules(id)
        ON DELETE CASCADE ON UPDATE CASCADE
);

CREATE UNIQUE INDEX IF NOT EXISTS uq_module_configs_active_per_module
    ON module_configs(module_id, is_active)
    WHERE is_active = 1;

CREATE INDEX IF NOT EXISTS idx_module_configs_module
    ON module_configs(module_id, created_at DESC);

-- =========================================================
-- slots: справочник лотков S01..S24 (MVP: 24)
-- =========================================================
CREATE TABLE IF NOT EXISTS slots (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    module_id         INTEGER NOT NULL,
    slot_code         TEXT    NOT NULL,
    slot_index        INTEGER NOT NULL CHECK (slot_index >= 1),
    is_enabled        INTEGER NOT NULL DEFAULT 1 CHECK (is_enabled IN (0, 1)),
    created_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (module_id) REFERENCES modules(id)
        ON DELETE CASCADE ON UPDATE CASCADE,
    UNIQUE (module_id, slot_code),
    UNIQUE (module_id, slot_index),
    CHECK (length(slot_code) = 3 AND substr(slot_code,1,1) = 'S')
);

CREATE INDEX IF NOT EXISTS idx_slots_module_code
    ON slots(module_id, slot_code);

-- =========================================================
-- reels: экземпляры катушек (barcode = тип, не уникальный экземпляр)
-- =========================================================
CREATE TABLE IF NOT EXISTS reels (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    reel_code         TEXT    NOT NULL, -- barcode типа катушки
    lot_no            TEXT,
    vendor_code       TEXT,
    status            TEXT    NOT NULL
                              CHECK (status IN ('in_cart','picked','consumed','quarantine','removed')),
    current_slot_id   INTEGER,          -- nullable: если вне тележки
    installed_by_user_id INTEGER,
    installed_at      DATETIME,
    removed_at        DATETIME,
    metadata_json     TEXT,
    created_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (current_slot_id) REFERENCES slots(id)
        ON DELETE SET NULL ON UPDATE CASCADE,
    FOREIGN KEY (installed_by_user_id) REFERENCES users(id)
        ON DELETE SET NULL ON UPDATE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_reels_code_status
    ON reels(reel_code, status);

CREATE INDEX IF NOT EXISTS idx_reels_current_slot
    ON reels(current_slot_id);

-- =========================================================
-- current_slot_state: текущее состояние занятости лотков
-- =========================================================
CREATE TABLE IF NOT EXISTS current_slot_state (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    slot_id           INTEGER NOT NULL UNIQUE,
    reel_id           INTEGER,
    occupancy_status  TEXT    NOT NULL
                              CHECK (occupancy_status IN ('empty','occupied','reserved','blocked')),
    reserved_for_op_id INTEGER,
    updated_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (slot_id) REFERENCES slots(id)
        ON DELETE CASCADE ON UPDATE CASCADE,
    FOREIGN KEY (reel_id) REFERENCES reels(id)
        ON DELETE SET NULL ON UPDATE CASCADE,
    FOREIGN KEY (reserved_for_op_id) REFERENCES replacement_operations(id)
        ON DELETE SET NULL ON UPDATE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_current_slot_state_reel
    ON current_slot_state(reel_id);

CREATE INDEX IF NOT EXISTS idx_current_slot_state_status
    ON current_slot_state(occupancy_status);

-- =========================================================
-- replacement_operations: операция замены катушки
-- =========================================================
CREATE TABLE IF NOT EXISTS replacement_operations (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    operation_uuid    TEXT    NOT NULL UNIQUE,
    status            TEXT    NOT NULL
                              CHECK (status IN ('pending','in_progress','completed','failed','cancelled')),
    old_reel_code     TEXT    NOT NULL,
    selected_new_reel_id INTEGER,
    target_slot_id    INTEGER,
    worker_user_id    INTEGER,
    started_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    completed_at      DATETIME,
    fail_reason_code  TEXT,
    context_json      TEXT,

    FOREIGN KEY (selected_new_reel_id) REFERENCES reels(id)
        ON DELETE SET NULL ON UPDATE CASCADE,
    FOREIGN KEY (target_slot_id) REFERENCES slots(id)
        ON DELETE SET NULL ON UPDATE CASCADE,
    FOREIGN KEY (worker_user_id) REFERENCES users(id)
        ON DELETE SET NULL ON UPDATE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_replacement_operations_status_created
    ON replacement_operations(status, started_at DESC);

CREATE INDEX IF NOT EXISTS idx_replacement_operations_old_reel_code
    ON replacement_operations(old_reel_code, started_at DESC);

-- =========================================================
-- replacement_operation_steps: пошаговая история операции
-- =========================================================
CREATE TABLE IF NOT EXISTS replacement_operation_steps (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    operation_id      INTEGER NOT NULL,
    step_no           INTEGER NOT NULL CHECK (step_no >= 1),
    step_name         TEXT    NOT NULL,
    step_status       TEXT    NOT NULL
                              CHECK (step_status IN ('started','completed','failed','skipped')),
    actor_user_id     INTEGER,
    payload_json      TEXT,
    created_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (operation_id) REFERENCES replacement_operations(id)
        ON DELETE CASCADE ON UPDATE CASCADE,
    FOREIGN KEY (actor_user_id) REFERENCES users(id)
        ON DELETE SET NULL ON UPDATE CASCADE,
    UNIQUE (operation_id, step_no)
);

CREATE INDEX IF NOT EXISTS idx_operation_steps_op_created
    ON replacement_operation_steps(operation_id, created_at);

-- =========================================================
-- system_events: события системы и ошибок
-- =========================================================
CREATE TABLE IF NOT EXISTS system_events (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    event_time        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    level             TEXT    NOT NULL
                              CHECK (level IN ('debug','info','warning','error','critical')),
    event_code        TEXT    NOT NULL,
    source            TEXT    NOT NULL, -- app/hw/db/ui/...
    message           TEXT    NOT NULL,
    operation_id      INTEGER,
    user_id           INTEGER,
    payload_json      TEXT,

    FOREIGN KEY (operation_id) REFERENCES replacement_operations(id)
        ON DELETE SET NULL ON UPDATE CASCADE,
    FOREIGN KEY (user_id) REFERENCES users(id)
        ON DELETE SET NULL ON UPDATE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_system_events_level_time
    ON system_events(level, event_time DESC);

CREATE INDEX IF NOT EXISTS idx_system_events_code_time
    ON system_events(event_code, event_time DESC);

-- =========================================================
-- users: роли worker/admin, для MVP admin по PIN
-- =========================================================
CREATE TABLE IF NOT EXISTS users (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    username          TEXT    NOT NULL UNIQUE,
    role              TEXT    NOT NULL CHECK (role IN ('worker','admin')),
    pin_hash          TEXT, -- для MVP хранится hash, не plaintext
    is_active         INTEGER NOT NULL DEFAULT 1 CHECK (is_active IN (0, 1)),
    created_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX IF NOT EXISTS idx_users_role_active
    ON users(role, is_active);

-- =========================================================
-- user_sessions: входы пользователей (аудит)
-- =========================================================
CREATE TABLE IF NOT EXISTS user_sessions (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    user_id           INTEGER NOT NULL,
    role_at_login     TEXT    NOT NULL CHECK (role_at_login IN ('worker','admin')),
    session_token     TEXT    NOT NULL UNIQUE,
    started_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    ended_at          DATETIME,
    auth_method       TEXT    NOT NULL CHECK (auth_method IN ('pin','auto','service')),
    status            TEXT    NOT NULL CHECK (status IN ('active','closed','expired','aborted')),
    client_info       TEXT,

    FOREIGN KEY (user_id) REFERENCES users(id)
        ON DELETE RESTRICT ON UPDATE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_user_sessions_user_started
    ON user_sessions(user_id, started_at DESC);

CREATE INDEX IF NOT EXISTS idx_user_sessions_status_started
    ON user_sessions(status, started_at DESC);

-- =========================================================
-- application_state: одиночный снимок runtime-состояния приложения
-- =========================================================
CREATE TABLE IF NOT EXISTS application_state (
    id                INTEGER PRIMARY KEY CHECK (id = 1),
    state_version     INTEGER NOT NULL DEFAULT 1,
    app_mode          TEXT    NOT NULL CHECK (app_mode IN ('idle','worker','admin','maintenance','error')),
    active_module_id  INTEGER,
    active_operation_id INTEGER,
    last_error_code   TEXT,
    ui_state_json     TEXT,
    updated_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (active_module_id) REFERENCES modules(id)
        ON DELETE SET NULL ON UPDATE CASCADE,
    FOREIGN KEY (active_operation_id) REFERENCES replacement_operations(id)
        ON DELETE SET NULL ON UPDATE CASCADE
);

-- Гарантируем наличие singleton-строки
INSERT OR IGNORE INTO application_state (id, state_version, app_mode)
VALUES (1, 1, 'idle');

-- =========================================================
-- recovery_state: состояние механизма восстановления
-- =========================================================
CREATE TABLE IF NOT EXISTS recovery_state (
    id                INTEGER PRIMARY KEY CHECK (id = 1),
    is_active         INTEGER NOT NULL DEFAULT 0 CHECK (is_active IN (0, 1)),
    last_checkpoint   TEXT,
    pending_operation_id INTEGER,
    details_json      TEXT,
    updated_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (pending_operation_id) REFERENCES replacement_operations(id)
        ON DELETE SET NULL ON UPDATE CASCADE
);

INSERT OR IGNORE INTO recovery_state (id, is_active)
VALUES (1, 0);

-- =========================================================
-- backward compatibility: event_log (если уже используется IEventLogger)
-- =========================================================
CREATE TABLE IF NOT EXISTS event_log (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    ts                DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    level             TEXT    NOT NULL,
    code              TEXT    NOT NULL,
    message           TEXT    NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_event_log_ts
    ON event_log(ts DESC);
