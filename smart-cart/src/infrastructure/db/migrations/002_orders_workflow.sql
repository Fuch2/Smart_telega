-- ===== src/infrastructure/db/migrations/002_orders_workflow.sql =====
-- Заказы и бизнес-состояние тележки по процессу из ТЗ 13.2.

PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS orders (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    external_order_id TEXT    NOT NULL UNIQUE,
    title             TEXT    NOT NULL DEFAULT '',
    priority          TEXT    NOT NULL DEFAULT '',
    duration_minutes  INTEGER NOT NULL DEFAULT 0 CHECK (duration_minutes >= 0),
    status            TEXT    NOT NULL DEFAULT 'LOADED'
                              CHECK (status IN ('LOADED','IN_PROGRESS','COMPLETED','CANCELLED')),
    created_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX IF NOT EXISTS idx_orders_status
    ON orders(status, updated_at);

CREATE TABLE IF NOT EXISTS order_items (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    order_id      INTEGER NOT NULL,
    barcode       TEXT    NOT NULL,
    material_type TEXT    NOT NULL DEFAULT 'reel',
    target_slot   INTEGER NOT NULL CHECK (target_slot >= 1),
    current_slot  INTEGER CHECK (current_slot IS NULL OR current_slot >= 1),
    status        TEXT    NOT NULL DEFAULT 'PENDING'
                          CHECK (status IN ('PENDING','PLACED','ISSUED','RETURNED','MISSING','WRONG_SLOT')),
    updated_at    DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    UNIQUE(order_id, barcode),
    FOREIGN KEY (order_id) REFERENCES orders(id)
        ON DELETE CASCADE ON UPDATE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_order_items_order_status
    ON order_items(order_id, status);

CREATE INDEX IF NOT EXISTS idx_order_items_barcode
    ON order_items(barcode);

CREATE TABLE IF NOT EXISTS cart_workflow (
    id               INTEGER PRIMARY KEY CHECK (id = 1),
    current_order_id INTEGER,
    state            TEXT    NOT NULL DEFAULT 'FREE'
                             CHECK (state IN (
                                 'FREE',
                                 'ORDER_LOADED',
                                 'PICKING_MATERIALS',
                                 'READY_FOR_FEEDER_PREP',
                                 'FEEDER_PREP',
                                 'READY_FOR_LINE',
                                 'ISSUING_TO_LINE',
                                 'ORDER_COMPLETED',
                                 'LEFTOVERS_DETECTED',
                                 'RETURNING_LEFTOVERS'
                             )),
    updated_at       DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (current_order_id) REFERENCES orders(id)
        ON DELETE SET NULL ON UPDATE CASCADE
);

INSERT INTO cart_workflow(id, current_order_id, state)
VALUES(1, NULL, 'FREE')
ON CONFLICT(id) DO NOTHING;
