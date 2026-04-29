-- ===== src/infrastructure/db/migrations/006_order_item_part_priority.sql =====
-- Добавляет производственный PartNumber, количество на плату и приоритет
-- частоты использования компонента.

ALTER TABLE order_items
    ADD COLUMN part_number TEXT NOT NULL DEFAULT '';

ALTER TABLE order_items
    ADD COLUMN required_quantity INTEGER NOT NULL DEFAULT 1
        CHECK (required_quantity >= 1);

ALTER TABLE order_items
    ADD COLUMN usage_priority INTEGER NOT NULL DEFAULT 3
        CHECK (usage_priority BETWEEN 1 AND 3);

CREATE INDEX IF NOT EXISTS idx_order_items_part_number
    ON order_items(order_id, part_number);
