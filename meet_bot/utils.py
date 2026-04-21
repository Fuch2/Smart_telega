import re
import logging
from datetime import datetime
from typing import Optional
import pytz

from config import TIMEZONE

logger = logging.getLogger(__name__)

TZ = pytz.timezone(TIMEZONE)

# ─── СООБЩЕНИЯ ────────────────────────────────────────────────────────────────

MESSAGES = {
    "ask_title": (
        "📝 Введи название встречи:"
    ),
    "ask_deadline": (
        "⏰ Укажи дедлайн для голосования.\n\n"
        "Поддерживаемые форматы:\n"
        "• <code>12 апреля 18:00</code>\n"
        "• <code>12.04 18:00</code>\n"
        "• <code>12.04.2025 18:00</code>"
    ),
    "ask_dates": (
        "📅 Выбери даты, когда ты свободен.\n"
        "Нажимай на даты — они будут подсвечиваться.\n"
        "Когда закончишь, нажми «✅ Готово»."
    ),
    "ask_slots_mode": (
        "🕐 Теперь выбери временные слоты.\n\n"
        "Режим «Одинаковое время» — одни слоты для всех дат.\n"
        "Режим «По датам» — для каждой даты свои слоты."
    ),
    "ask_slots_for_date": (
        "🕐 Выбери слоты для {date}:"
    ),
    "no_dates_selected": (
        "⚠️ Ты не выбрал ни одной даты. Выбери хотя бы одну."
    ),
    "no_slots_selected": (
        "⚠️ Ты не выбрал ни одного временного слота. Выбери хотя бы один."
    ),
    "ask_custom_time_single": (
        "🕐 Отправь своё время для даты {date} в формате <code>ЧЧ:ММ</code>.\n"
        "Например: <code>16:00</code>"
    ),
    "ask_custom_time_multiple": (
        "🕐 Отправь своё время для отмеченных дат ({dates}) в формате <code>ЧЧ:ММ</code>.\n"
        "Например: <code>16:00</code>"
    ),
    "invalid_custom_time": (
        "❌ Не удалось распознать время. Используй формат <code>ЧЧ:ММ</code>, например <code>16:00</code>."
    ),
    "custom_time_no_dates": (
        "⚠️ Сначала выбери дату или отметь даты, к которым нужно добавить время."
    ),
    "custom_time_added": (
        "✅ Добавил время {time}. Теперь этот слот виден всем участникам этой встречи."
    ),
    "invalid_deadline": (
        "❌ Не удалось распознать дату. Попробуй так:\n"
        "• <code>12 апреля 18:00</code>\n"
        "• <code>12.04 18:00</code>\n"
        "• <code>12.04.2025 18:00</code>"
    ),
    "deadline_in_past": (
        "❌ Дедлайн не может быть в прошлом. Укажи будущую дату."
    ),
    "meeting_created": (
        "✅ Встреча создана! Анонс отправлен в чат."
    ),
    "private_welcome": (
        "👋 Привет! Я бот для организации встреч.\n"
        "Здесь можно открыть свой календарь, проверить интеграции и перейти к активным встречам."
    ),
    "personal_calendar_empty": (
        "📭 У тебя пока нет активных встреч в личном календаре."
    ),
    "personal_calendar_header": (
        "📅 Твой календарь встреч:\n{lines}"
    ),
    "integrations_header": (
        "🔗 Интеграции\n\n"
        "Google Calendar: {google_status}\n"
        "Todoist: {todoist_status}\n\n"
        "Экран уже подготовлен. Подключение OAuth добавим следующим этапом."
    ),
    "integration_not_connected": (
        "не подключено"
    ),
    "integration_connected": (
        "подключено"
    ),
    "announce_template": (
        "📅 Встреча: <b>{title}</b>\n"
        "⏰ Дедлайн: {deadline}\n"
        "👥 Участников отметилось: {count}\n\n"
        "👉 Нажми кнопку ниже, чтобы выбрать удобное время"
    ),
    "vote_intro": (
        "📅 Встреча: <b>{title}</b>\n"
        "⏰ Дедлайн: {deadline}\n\n"
        "Выбери даты, когда ты свободен:"
    ),
    "vote_saved": (
        "✅ Готово! Твой выбор сохранён.\n"
        "Результаты будут объявлены после дедлайна."
    ),
    "votes_overview_empty": (
        "👥 Пока никто не сохранил свой выбор."
    ),
    "votes_overview_template": (
        "👥 Кто уже отметил время:\n{lines}"
    ),
    "deadline_passed": (
        "⏰ Дедлайн истёк, изменения недоступны."
    ),
    "meeting_not_found": (
        "❌ Встреча не найдена."
    ),
    "meeting_closed": (
        "🔒 Эта встреча уже закрыта."
    ),
    "no_active_meetings": (
        "📭 В этом чате нет активных встреч."
    ),
    "meeting_cancelled": (
        "❌ Встреча «{title}» отменена."
    ),
    "cancel_no_permission": (
        "🚫 Только создатель встречи или администратор может её отменить."
    ),
    "cancel_no_active": (
        "📭 В этом чате нет активных встреч для отмены."
    ),
    "select_meeting_to_cancel": (
        "Выбери встречу для отмены:"
    ),
    "no_slots_in_meeting": (
        "⚠️ В этой встрече пока нет слотов."
    ),
    "choose_time_header": (
        "🗓 Встреча: <b>{title}</b>\n"
        "⏰ Дедлайн: {deadline}\n\n"
        "Отмечай удобное время (✅ — выбрано), затем нажми «Готово»:"
    ),
    "results_no_participants": (
        "📊 Встреча «{title}»\n\n"
        "Пока никто не отметил своё время."
    ),
    "results_no_common": (
        "📊 Встреча «{title}»\n\n"
        "👥 Участников: {total}\n\n"
        "👤 Голосовали: {participants}\n\n"
        "😔 Общего удобного времени не найдено."
    ),
    "results_found": (
        "📊 Встреча «{title}»\n\n"
        "👥 Участников: {total}\n\n"
        "👤 Голосовали: {participants}\n\n"
        "✅ Подходящее время для всех:\n{slots}"
    ),
    "results_confirmed": (
        "✅ Встреча «{title}» состоится!\n\n"
        "👥 Участников: {total}\n\n"
        "👤 Голосовали: {participants}\n\n"
        "📅 Финальное время: {slot}"
    ),
}


# ─── МЕСЯЦЫ ───────────────────────────────────────────────────────────────────

MONTH_MAP = {
    "января": 1, "февраля": 2, "марта": 3, "апреля": 4,
    "мая": 5, "июня": 6, "июля": 7, "августа": 8,
    "сентября": 9, "октября": 10, "ноября": 11, "декабря": 12,
}

WEEKDAY_SHORT = ["пн", "вт", "ср", "чт", "пт", "сб", "вс"]


# ─── ПАРСИНГ ДЕДЛАЙНА ─────────────────────────────────────────────────────────

def _pick_year(day: int, month: int, now: datetime) -> int:
    """
    Берёт текущий год.
    Исключение: сегодня 31 декабря и вводится тоже 31 декабря → следующий год.
    """
    year = now.year
    if now.month == 12 and now.day == 31 and month == 12 and day == 31:
        year += 1
    return year


def parse_deadline(text: str) -> Optional[datetime]:
    """
    Поддерживает форматы:
      - '25.12.2024 18:00'  — с явным годом
      - '25.12 18:00'       — год подставляется автоматически
      - '25 декабря 18:00'  — год подставляется автоматически
    """
    text = text.strip().lower()
    now = datetime.now(TZ)

    # Формат 1: ДД.ММ.ГГГГ ЧЧ:ММ
    m = re.match(r"^(\d{1,2})\.(\d{2})\.(\d{4})\s+(\d{1,2}):(\d{2})$", text)
    if m:
        day, month, year, hour, minute = (int(x) for x in m.groups())
        try:
            return TZ.localize(datetime(year, month, day, hour, minute))
        except ValueError:
            return None

    # Формат 2: ДД.ММ ЧЧ:ММ
    m = re.match(r"^(\d{1,2})\.(\d{2})\s+(\d{1,2}):(\d{2})$", text)
    if m:
        day, month, hour, minute = (int(x) for x in m.groups())
        year = _pick_year(day, month, now)
        try:
            return TZ.localize(datetime(year, month, day, hour, minute))
        except ValueError:
            return None

    # Формат 3: ДД <месяц_словом> ЧЧ:ММ
    m = re.match(r"^(\d{1,2})\s+([а-яё]+)\s+(\d{1,2}):(\d{2})$", text)
    if m:
        day = int(m.group(1))
        month = MONTH_MAP.get(m.group(2))
        if month is None:
            return None
        hour, minute = int(m.group(3)), int(m.group(4))
        year = _pick_year(day, month, now)
        try:
            return TZ.localize(datetime(year, month, day, hour, minute))
        except ValueError:
            return None

    return None


def parse_time_input(text: str) -> Optional[str]:
    """Парсит время в формате HH:MM или H:MM."""
    text = text.strip()
    match = re.match(r"^(\d{1,2}):(\d{2})$", text)
    if not match:
        return None

    hour, minute = (int(x) for x in match.groups())
    if not (0 <= hour <= 23 and 0 <= minute <= 59):
        return None

    return f"{hour:02d}:{minute:02d}"


# ─── УТИЛИТЫ ВРЕМЕНИ ──────────────────────────────────────────────────────────

def now_moscow() -> datetime:
    """Возвращает текущее время в Europe/Moscow."""
    return datetime.now(TZ)


def format_datetime_moscow(dt: datetime) -> str:
    """Форматирует datetime в строку 'ДД.ММ.ГГГГ ЧЧ:ММ'."""
    if dt.tzinfo is None:
        dt = TZ.localize(dt)
    return dt.strftime("%d.%m.%Y %H:%M")


def parse_slot_datetime(slot_str: str) -> datetime:
    """Парсит строку слота 'YYYY-MM-DD HH:MM' в timezone-aware datetime."""
    naive = datetime.strptime(slot_str, "%Y-%m-%d %H:%M")
    return TZ.localize(naive)


def slot_to_display(slot_str: str) -> str:
    """Преобразует 'YYYY-MM-DD HH:MM' → 'ДД.ММ.ГГГГ ЧЧ:ММ'."""
    dt = parse_slot_datetime(slot_str)
    return dt.strftime("%d.%m.%Y %H:%M")


def slot_to_compact_display(slot_str: str) -> str:
    """Преобразует слот в формат 'чт 10.04 18:00'."""
    dt = parse_slot_datetime(slot_str)
    return f"{WEEKDAY_SHORT[dt.weekday()]} {dt.strftime('%d.%m %H:%M')}"


def format_participant_names(participants: list[dict]) -> str:
    names = [p["username"] for p in participants if p.get("username")]
    return ", ".join(names) if names else "—"


def format_participant_votes(votes: list[dict]) -> str:
    if not votes:
        return MESSAGES["votes_overview_empty"]

    grouped: dict[tuple[int, str], list[str]] = {}
    order: list[tuple[int, str]] = []

    for row in votes:
        user_id = row["user_id"]
        username = row.get("username") or f"User {user_id}"
        key = (user_id, username)
        if key not in grouped:
            grouped[key] = []
            order.append(key)

        slot_datetime = row.get("slot_datetime")
        if slot_datetime:
            grouped[key].append(slot_to_compact_display(slot_datetime))

    lines = []
    for key in order:
        username = key[1]
        slots = grouped[key]
        slots_text = ", ".join(slots) if slots else "пока без выбранного времени"
        lines.append(f"• {username} — {slots_text}")

    return MESSAGES["votes_overview_template"].format(lines="\n".join(lines))
