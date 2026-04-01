import os
from dotenv import load_dotenv

load_dotenv()

BOT_TOKEN: str = os.getenv("BOT_TOKEN", "")

if not BOT_TOKEN:
    raise ValueError("BOT_TOKEN не задан. Укажи его в .env файле или переменной окружения.")

TIMEZONE = "Europe/Moscow"

DB_PATH = "meet_bot.db"

SCHEDULER_INTERVAL = 60  # секунды между проверками дедлайнов

# Временные слоты для будних дней (18:00–23:00)
WEEKDAY_HOURS = list(range(18, 24))

# Временные слоты для выходных дней (14:00–23:00)
WEEKEND_HOURS = list(range(14, 24))
