-- ===== tests/test_data/db/reset_db.sql =====
-- Исправлено: приведено в соответствие со схемой 001_init.sql
-- Очищает все рабочие таблицы, оставляя schema_migrations нетронутой
PRAGMA foreign_keys = OFF;
BEGIN TRANSACTION;

DELETE FROM event_log;
DELETE FROM operations;
DELETE FROM reels;
DELETE FROM slot_states;
DELETE FROM modules;

COMMIT;
PRAGMA foreign_keys = ON;
