PRAGMA foreign_keys = OFF;
BEGIN TRANSACTION;

DELETE FROM replacement_operation_steps;
DELETE FROM replacement_operations;
DELETE FROM system_events;
DELETE FROM current_slot_state;
DELETE FROM reels;
DELETE FROM slots;
DELETE FROM module_configs;
DELETE FROM modules;
DELETE FROM user_sessions;
DELETE FROM users;

UPDATE application_state
SET state_version = 1,
    app_mode = 'idle',
    active_module_id = NULL,
    active_operation_id = NULL,
    last_error_code = NULL,
    ui_state_json = NULL,
    updated_at = CURRENT_TIMESTAMP
WHERE id = 1;

UPDATE recovery_state
SET is_active = 0,
    last_checkpoint = NULL,
    pending_operation_id = NULL,
    details_json = NULL,
    updated_at = CURRENT_TIMESTAMP
WHERE id = 1;

COMMIT;
PRAGMA foreign_keys = ON;
