-- ===== src/infrastructure/db/migrations/003_module_scoped_workflow.sql =====
-- Привязывает orders и cart_workflow к module_id, чтобы один хост мог
-- различать разные модули по RFID-метке.

PRAGMA foreign_keys = OFF;

CREATE TABLE orders_new (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    module_id         INTEGER NOT NULL DEFAULT 1,
    external_order_id TEXT    NOT NULL,
    title             TEXT    NOT NULL DEFAULT '',
    priority          TEXT    NOT NULL DEFAULT '',
    duration_minutes  INTEGER NOT NULL DEFAULT 0 CHECK (duration_minutes >= 0),
    status            TEXT    NOT NULL DEFAULT 'LOADED'
                              CHECK (status IN ('LOADED','IN_PROGRESS','COMPLETED','CANCELLED')),
    created_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(module_id, external_order_id)
);

INSERT INTO orders_new(
    id,
    module_id,
    external_order_id,
    title,
    priority,
    duration_minutes,
    status,
    created_at,
    updated_at
)
SELECT
    id,
    1,
    external_order_id,
    title,
    priority,
    duration_minutes,
    status,
    created_at,
    updated_at
FROM orders;

DROP TABLE orders;
ALTER TABLE orders_new RENAME TO orders;

CREATE INDEX IF NOT EXISTS idx_orders_status
    ON orders(module_id, status, updated_at);

CREATE TABLE cart_workflow_new (
    module_id        INTEGER PRIMARY KEY,
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

INSERT INTO cart_workflow_new(module_id, current_order_id, state, updated_at)
SELECT
    1,
    current_order_id,
    state,
    updated_at
FROM cart_workflow;

DROP TABLE cart_workflow;
ALTER TABLE cart_workflow_new RENAME TO cart_workflow;

PRAGMA foreign_keys = ON;
