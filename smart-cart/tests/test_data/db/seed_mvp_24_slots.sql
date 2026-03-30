-- ===== tests/test_data/db/seed_mvp_24_slots.sql =====
-- Исправлено: приведено в соответствие со схемой 001_init.sql
-- Таблицы: modules, slot_states, reels (пусто), operations (пусто)
PRAGMA foreign_keys = ON;
BEGIN IMMEDIATE TRANSACTION;

-- 1. Модуль MVP
INSERT OR IGNORE INTO modules (id, serial, slot_count, firmware, status)
VALUES (1, 'MVP-SN-0001', 24, '0.1.0', 'ONLINE');

-- 2. Состояния слотов S01..S24 — все свободны
INSERT OR IGNORE INTO slot_states (module_id, slot_index, state) VALUES
  (1,  1, 'FREE'), (1,  2, 'FREE'), (1,  3, 'FREE'), (1,  4, 'FREE'),
  (1,  5, 'FREE'), (1,  6, 'FREE'), (1,  7, 'FREE'), (1,  8, 'FREE'),
  (1,  9, 'FREE'), (1, 10, 'FREE'), (1, 11, 'FREE'), (1, 12, 'FREE'),
  (1, 13, 'FREE'), (1, 14, 'FREE'), (1, 15, 'FREE'), (1, 16, 'FREE'),
  (1, 17, 'FREE'), (1, 18, 'FREE'), (1, 19, 'FREE'), (1, 20, 'FREE'),
  (1, 21, 'FREE'), (1, 22, 'FREE'), (1, 23, 'FREE'), (1, 24, 'FREE');

COMMIT;
