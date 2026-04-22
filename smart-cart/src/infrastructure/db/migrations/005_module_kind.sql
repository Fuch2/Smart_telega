-- Тип модуля нужен UI, чтобы на разных этапах маршрута показывать
-- состояние нужной физической кассеты: катушки или питатели.

ALTER TABLE modules
ADD COLUMN kind TEXT NOT NULL DEFAULT 'REEL'
CHECK (kind IN ('REEL','FEEDER','UNKNOWN'));
