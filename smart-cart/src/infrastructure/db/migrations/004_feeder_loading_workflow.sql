-- ===== src/infrastructure/db/migrations/004_feeder_loading_workflow.sql =====
-- Добавляет этап LOADING_FEEDERS в workflow тележки.

PRAGMA foreign_keys = OFF;

CREATE TABLE cart_workflow_new (
    module_id        INTEGER PRIMARY KEY,
    current_order_id INTEGER,
    state            TEXT    NOT NULL DEFAULT 'FREE'
                             CHECK (state IN (
                                 'FREE',
                                 'ORDER_LOADED',
                                 'LOADING_FEEDERS',
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
SELECT module_id, current_order_id, state, updated_at
FROM cart_workflow;

DROP TABLE cart_workflow;
ALTER TABLE cart_workflow_new RENAME TO cart_workflow;

PRAGMA foreign_keys = ON;
