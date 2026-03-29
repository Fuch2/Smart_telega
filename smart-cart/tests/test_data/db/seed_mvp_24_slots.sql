PRAGMA foreign_keys = ON;
BEGIN IMMEDIATE TRANSACTION;

-- 1) Users (MVP)
INSERT OR IGNORE INTO users (id, username, role, pin_hash, is_active)
VALUES
  (1, 'worker_default', 'worker', NULL, 1),
  (2, 'admin_default',  'admin',  'mvp_pin_hash_change_me', 1);

-- 2) Active module
INSERT OR IGNORE INTO modules (
    id, serial_number, module_type, module_name, firmware_version, is_active
) VALUES (
    1, 'MVP-SN-0001', 'tray24', 'Tray 24 MVP', '0.1.0', 1
);

-- 3) Module config
INSERT OR IGNORE INTO module_configs (
    id, module_id, config_version, profile_name, slot_count, config_json, is_active
) VALUES (
    1,
    1,
    1,
    'tray24',
    24,
    '{"profile":"tray24","slots":["S01","S02","S03","S04","S05","S06","S07","S08","S09","S10","S11","S12","S13","S14","S15","S16","S17","S18","S19","S20","S21","S22","S23","S24"]}',
    1
);

-- 4) Slots S01..S24
INSERT OR IGNORE INTO slots (id, module_id, slot_code, slot_index, is_enabled) VALUES
  (1, 1, 'S01',  1, 1),  (2, 1, 'S02',  2, 1),  (3, 1, 'S03',  3, 1),  (4, 1, 'S04',  4, 1),
  (5, 1, 'S05',  5, 1),  (6, 1, 'S06',  6, 1),  (7, 1, 'S07',  7, 1),  (8, 1, 'S08',  8, 1),
  (9, 1, 'S09',  9, 1),  (10,1, 'S10', 10, 1),  (11,1, 'S11', 11, 1),  (12,1, 'S12', 12, 1),
  (13,1, 'S13', 13, 1),  (14,1, 'S14', 14, 1),  (15,1, 'S15', 15, 1),  (16,1, 'S16', 16, 1),
  (17,1, 'S17', 17, 1),  (18,1, 'S18', 18, 1),  (19,1, 'S19', 19, 1),  (20,1, 'S20', 20, 1),
  (21,1, 'S21', 21, 1),  (22,1, 'S22', 22, 1),  (23,1, 'S23', 23, 1),  (24,1, 'S24', 24, 1);

-- 5) Current slot state -> empty
INSERT OR IGNORE INTO current_slot_state (slot_id, reel_id, occupancy_status)
SELECT s.id, NULL, 'empty'
FROM slots s
WHERE s.module_id = 1;

-- 6) Singleton state rows (на случай запуска seed отдельно от migration init)
INSERT OR IGNORE INTO application_state (id, state_version, app_mode)
VALUES (1, 1, 'idle');

INSERT OR IGNORE INTO recovery_state (id, is_active)
VALUES (1, 0);

COMMIT;
