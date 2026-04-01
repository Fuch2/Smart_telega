import calendar
from datetime import date, datetime

import pytz
from aiogram.filters.callback_data import CallbackData
from aiogram.types import InlineKeyboardButton, InlineKeyboardMarkup
from aiogram.utils.keyboard import InlineKeyboardBuilder

from config import TIMEZONE, WEEKDAY_HOURS, WEEKEND_HOURS

TZ = pytz.timezone(TIMEZONE)


# ─── CALLBACK DATA ────────────────────────────────────────────────────────────

class CalendarCallback(CallbackData, prefix="cal"):
    action: str
    year: int = 0
    month: int = 0
    day: int = 0


class SlotActionCallback(CallbackData, prefix="slotact"):
    """mode / done / noop / prev / next / toggle_date"""
    action: str
    value: int = 0   # используется для prev/next (индекс) и toggle_date (индекс даты)


class SlotCallback(CallbackData, prefix="slot"):
    """Toggle временного слота."""
    slot_key: str


class MeetingActionCallback(CallbackData, prefix="meet"):
    action: str
    meeting_id: int


# ─── HELPERS ──────────────────────────────────────────────────────────────────

def encode_slot_key(slot_key: str) -> str:
    """Заменяет ':' на '_' во всей строке."""
    return slot_key.replace(":", "_")


def decode_slot_key(slot_key: str) -> str:
    """Заменяет '_' на ':' во всей строке."""
    return slot_key.replace("_", ":")


def slot_to_display(slot_str: str) -> str:
    dt = datetime.strptime(slot_str, "%Y-%m-%d %H:%M")
    return dt.strftime("%d.%m.%Y %H:%M")


# ─── КАЛЕНДАРЬ ────────────────────────────────────────────────────────────────

WEEKDAY_NAMES = ["Пн", "Вт", "Ср", "Чт", "Пт", "Сб", "Вс"]
MONTH_NAMES = [
    "", "Январь", "Февраль", "Март", "Апрель", "Май", "Июнь",
    "Июль", "Август", "Сентябрь", "Октябрь", "Ноябрь", "Декабрь",
]


def build_calendar(year: int, month: int, selected_dates: set) -> InlineKeyboardMarkup:
    builder = InlineKeyboardBuilder()

    builder.row(
        InlineKeyboardButton(
            text="◀️",
            callback_data=CalendarCallback(action="prev", year=year, month=month).pack(),
        ),
        InlineKeyboardButton(
            text=f"{MONTH_NAMES[month]} {year}",
            callback_data=CalendarCallback(action="ignore", year=year, month=month).pack(),
        ),
        InlineKeyboardButton(
            text="▶️",
            callback_data=CalendarCallback(action="next", year=year, month=month).pack(),
        ),
    )

    builder.row(*[
        InlineKeyboardButton(
            text=name,
            callback_data=CalendarCallback(action="ignore", year=year, month=month).pack(),
        )
        for name in WEEKDAY_NAMES
    ])

    cal = calendar.monthcalendar(year, month)
    today = date.today()

    for week in cal:
        row_buttons = []
        for day_num in week:
            if day_num == 0:
                row_buttons.append(InlineKeyboardButton(
                    text=" ",
                    callback_data=CalendarCallback(
                        action="ignore", year=year, month=month
                    ).pack(),
                ))
            else:
                current_date = date(year, month, day_num)
                date_str = current_date.strftime("%Y-%m-%d")
                is_selected = date_str in selected_dates
                is_past = current_date < today

                if is_past:
                    label = f"·{day_num}·"
                    action = "ignore"
                elif is_selected:
                    label = f"✅{day_num}"
                    action = "day"
                else:
                    label = str(day_num)
                    action = "day"

                row_buttons.append(InlineKeyboardButton(
                    text=label,
                    callback_data=CalendarCallback(
                        action=action, year=year, month=month, day=day_num,
                    ).pack(),
                ))
        builder.row(*row_buttons)

    builder.row(InlineKeyboardButton(
        text="✅ Готово",
        callback_data=CalendarCallback(action="done", year=year, month=month).pack(),
    ))

    return builder.as_markup()


# ─── СЛОТЫ ────────────────────────────────────────────────────────────────────

def _weekday_ru(weekday: int) -> str:
    return ["пн", "вт", "ср", "чт", "пт", "сб", "вс"][weekday]


def build_slots_keyboard(
    dates: list[str],
    selected_slots: dict[str, set[str]],  # {date_str: {"HH:MM", ...}}
    mode: str,                             # "same" | "per_date"
    current_date_index: int = 0,
    same_time_dates: set[int] | None = None,  # индексы дат, выбранных для "same"
) -> InlineKeyboardMarkup:
    """
    mode="same"     — показываем чекбоксы дат + слоты; выбранные даты получат одно время
    mode="per_date" — идём по датам по одной, у каждой свои слоты
    """
    if same_time_dates is None:
        same_time_dates = set()

    builder = InlineKeyboardBuilder()
    total = len(dates)

    # ── Кнопка переключения режима ────────────────────────────────────────────
    mode_label = "🔄 Режим: По датам" if mode == "same" else "🔄 Режим: Одинаковое время"
    builder.row(InlineKeyboardButton(
        text=mode_label,
        callback_data=SlotActionCallback(action="mode").pack(),
    ))

    # ═════════════════════════════════════════════════════════════════════════
    if mode == "same":
        # ── Шаг 1: выбираем даты, которым назначим одинаковое время ──────────
        builder.row(InlineKeyboardButton(
            text="📋 Отметь даты с одинаковым временем:",
            callback_data=SlotActionCallback(action="noop").pack(),
        ))

        for i, date_str in enumerate(dates):
            d = datetime.strptime(date_str, "%Y-%m-%d")
            is_checked = i in same_time_dates
            # Показываем уже выбранные слоты для этой даты (если есть)
            slots_for_date = selected_slots.get(date_str, set())
            slots_preview = " · ".join(sorted(slots_for_date)) if slots_for_date else ""
            label = (
                f"{'✅' if is_checked else '☐'} "
                f"{d.strftime('%d.%m')} ({_weekday_ru(d.weekday())})"
                + (f"  {slots_preview}" if slots_preview else "")
            )
            builder.row(InlineKeyboardButton(
                text=label,
                callback_data=SlotActionCallback(
                    action="toggle_date", value=i
                ).pack(),
            ))

        # ── Шаг 2: слоты (показываем только если хоть одна дата отмечена) ────
        if same_time_dates:
            builder.row(InlineKeyboardButton(
                text="⬇️ Выбери время для отмеченных дат:",
                callback_data=SlotActionCallback(action="noop").pack(),
            ))

            # Часы — объединение по отмеченным датам
            all_hours: set[int] = set()
            for i in same_time_dates:
                d = datetime.strptime(dates[i], "%Y-%m-%d")
                hours = WEEKEND_HOURS if d.weekday() >= 5 else WEEKDAY_HOURS
                all_hours.update(hours)

            # Текущие общие слоты — берём из первой отмеченной даты как эталон
            first_date = dates[next(iter(sorted(same_time_dates)))]
            current_same_slots = selected_slots.get(f"__same_{first_date}", set())
            # Используем специальный ключ __same для хранения "общего" выбора
            # Реальный ключ в selected_slots для режима same: "__same"
            current_same_slots = selected_slots.get("__same__", set())

            row_slots = []
            for hour in sorted(all_hours):
                slot_key = f"{hour:02d}:00"
                safe_key = encode_slot_key(slot_key)
                is_selected = slot_key in current_same_slots
                label = f"{'✅ ' if is_selected else ''}{hour:02d}:00"
                row_slots.append(InlineKeyboardButton(
                    text=label,
                    callback_data=SlotCallback(slot_key=safe_key).pack(),
                ))
                if len(row_slots) == 3:
                    builder.row(*row_slots)
                    row_slots = []
            if row_slots:
                builder.row(*row_slots)

            builder.row(InlineKeyboardButton(
                text="✅ Применить к отмеченным датам",
                callback_data=SlotActionCallback(action="apply_same").pack(),
            ))

    # ═════════════════════════════════════════════════════════════════════════
    else:  # mode == "per_date"
        date_str = dates[current_date_index]
        d = datetime.strptime(date_str, "%Y-%m-%d")

        # Заголовок с навигацией
        prev_label = "◀️" if current_date_index > 0 else "·"
        next_label = "▶️" if current_date_index < total - 1 else "·"

        builder.row(
            InlineKeyboardButton(
                text=prev_label,
                callback_data=SlotActionCallback(
                    action="prev", value=current_date_index - 1
                ).pack() if current_date_index > 0
                else SlotActionCallback(action="noop").pack(),
            ),
            InlineKeyboardButton(
                text=f"📅 {d.strftime('%d.%m.%Y')} ({_weekday_ru(d.weekday()).upper()})  {current_date_index + 1}/{total}",
                callback_data=SlotActionCallback(action="noop").pack(),
            ),
            InlineKeyboardButton(
                text=next_label,
                callback_data=SlotActionCallback(
                    action="next", value=current_date_index + 1
                ).pack() if current_date_index < total - 1
                else SlotActionCallback(action="noop").pack(),
            ),
        )

        is_weekend = d.weekday() >= 5
        hours = WEEKEND_HOURS if is_weekend else WEEKDAY_HOURS
        slots_for_date = selected_slots.get(date_str, set())

        row_slots = []
        for hour in hours:
            slot_key = f"{hour:02d}:00"
            safe_key = encode_slot_key(slot_key)
            is_selected = slot_key in slots_for_date
            label = f"{'✅ ' if is_selected else ''}{hour:02d}:00"
            row_slots.append(InlineKeyboardButton(
                text=label,
                callback_data=SlotCallback(slot_key=safe_key).pack(),
            ))
            if len(row_slots) == 3:
                builder.row(*row_slots)
                row_slots = []
        if row_slots:
            builder.row(*row_slots)

    # ── Кнопка «Готово» ───────────────────────────────────────────────────────
    builder.row(InlineKeyboardButton(
        text="✅ Готово",
        callback_data=SlotActionCallback(action="done").pack(),
    ))

    return builder.as_markup()


# ─── АНОНС ────────────────────────────────────────────────────────────────────

def build_announce_keyboard(meeting_id: int, bot_username: str) -> InlineKeyboardMarkup:
    builder = InlineKeyboardBuilder()
    builder.row(InlineKeyboardButton(
        text="🗓 Выбрать время",
        url=f"https://t.me/{bot_username}?start=vote_{meeting_id}",
    ))
    return builder.as_markup()


# ─── СПИСОК ВСТРЕЧ ────────────────────────────────────────────────────────────

def build_meetings_list_keyboard(meetings: list, bot_username: str) -> InlineKeyboardMarkup:
    builder = InlineKeyboardBuilder()
    for meeting in meetings:
        builder.row(InlineKeyboardButton(
            text=f"📅 {meeting['title']}",
            callback_data=MeetingActionCallback(
                action="vote", meeting_id=meeting["id"]
            ).pack(),
        ))
        builder.row(InlineKeyboardButton(
            text="🗓 Выбрать время",
            url=f"https://t.me/{bot_username}?start=vote_{meeting['id']}",
        ))
    return builder.as_markup()
