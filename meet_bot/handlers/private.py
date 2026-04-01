import logging
from datetime import datetime

import pytz
from aiogram import Router, F, Bot
from aiogram.filters import CommandStart
from aiogram.fsm.context import FSMContext
from aiogram.fsm.state import State, StatesGroup
from aiogram.types import Message, CallbackQuery

from config import TIMEZONE
from database import (
    get_meeting,
    add_user_slot,
    delete_user_slots,
    get_user_slots,
    upsert_participant,
    count_participants,
    save_message_id,
    create_meeting,
)
from keyboards.inline import (
    build_calendar,
    build_slots_keyboard,
    build_announce_keyboard,
    CalendarCallback,
    SlotCallback,
    SlotActionCallback,
    decode_slot_key,
)
from utils import (
    MESSAGES,
    now_moscow,
    format_datetime_moscow,
    parse_deadline,
)

logger = logging.getLogger(__name__)
router = Router()
TZ = pytz.timezone(TIMEZONE)


# ─── FSM СОСТОЯНИЯ ────────────────────────────────────────────────────────────

class MeetingCreation(StatesGroup):
    waiting_title    = State()
    waiting_deadline = State()


class VoteFlow(StatesGroup):
    choosing_dates = State()
    choosing_slots = State()


# ─── HELPERS ──────────────────────────────────────────────────────────────────

def _slots_dict_to_state(slots_dict: dict[str, set[str]]) -> dict[str, list[str]]:
    """set → list для сохранения в FSM."""
    return {k: list(v) for k, v in slots_dict.items()}


def _slots_dict_from_state(raw: dict) -> dict[str, set[str]]:
    """list → set при чтении из FSM."""
    return {k: set(v) for k, v in raw.items()}


def _restore_slots_dict(saved_slots: list[str]) -> dict[str, set[str]]:
    """
    Восстанавливает slots_dict из плоского списка 'YYYY-MM-DD HH:MM'
    (используется при повторном входе в голосование).
    """
    result: dict[str, set[str]] = {}
    for slot_str in saved_slots:
        parts = slot_str.split(" ", 1)
        if len(parts) == 2:
            d_str, t_str = parts
            result.setdefault(d_str, set()).add(t_str)
    return result


# ─── /start ───────────────────────────────────────────────────────────────────

@router.message(CommandStart(), F.chat.type == "private")
async def cmd_start_private(message: Message, state: FSMContext, bot: Bot) -> None:
    args    = message.text.split(maxsplit=1)
    payload = args[1] if len(args) >= 2 else ""

    # ── Создание встречи ──────────────────────────────────────────────────────
    if payload.startswith("newmeeting_"):
        parts = payload.replace("newmeeting_", "").split("_")
        try:
            group_chat_id = int(parts[0])
            invite_msg_id = int(parts[1]) if len(parts) > 1 else None
        except ValueError:
            await message.answer("❌ Неверная ссылка.")
            return

        if invite_msg_id:
            try:
                await bot.delete_message(
                    chat_id=group_chat_id,
                    message_id=invite_msg_id,
                )
            except Exception:
                pass

        await state.clear()
        await state.update_data(
            group_chat_id=group_chat_id,
            created_by=message.from_user.id,
        )
        await message.answer(MESSAGES["ask_title"])
        await state.set_state(MeetingCreation.waiting_title)
        return

    # ── Голосование за слоты ──────────────────────────────────────────────────
    if payload.startswith("vote_"):
        try:
            meeting_id = int(payload.replace("vote_", ""))
        except ValueError:
            await message.answer(MESSAGES["meeting_not_found"])
            return

        meeting = await get_meeting(meeting_id)
        if not meeting:
            await message.answer(MESSAGES["meeting_not_found"])
            return
        if meeting["status"] != "active":
            await message.answer(MESSAGES["meeting_closed"])
            return

        deadline_dt = datetime.fromisoformat(meeting["deadline"])
        if now_moscow() > deadline_dt:
            await message.answer(MESSAGES["deadline_passed"])
            return

        # Восстанавливаем ранее сохранённые слоты из БД
        saved_slots  = await get_user_slots(meeting_id, message.from_user.id)
        saved_dates  = sorted(set(s.split(" ")[0] for s in saved_slots))
        slots_dict   = _restore_slots_dict(saved_slots)

        await state.clear()
        await state.update_data(
            meeting_id=meeting_id,
            selected_dates=saved_dates,
            slots_dict=_slots_dict_to_state(slots_dict),
            slot_mode="per_date",
            current_date_index=0,
            same_time_dates=[],
            same_slots=[],
        )

        deadline_display = format_datetime_moscow(deadline_dt)
        await message.answer(
            MESSAGES["vote_intro"].format(
                title=meeting["title"],
                deadline=deadline_display,
            )
        )

        now = now_moscow()
        kb  = build_calendar(now.year, now.month, set(saved_dates))
        await message.answer(MESSAGES["ask_dates"], reply_markup=kb)
        await state.set_state(VoteFlow.choosing_dates)
        return

    # ── /start без payload ────────────────────────────────────────────────────
    await message.answer(
        "👋 Привет! Я бот для организации встреч.\n"
        "Перейди в групповой чат и нажми кнопку «Создать встречу» или «Выбрать время»."
    )


# ─── СОЗДАНИЕ: название ───────────────────────────────────────────────────────

@router.message(MeetingCreation.waiting_title, F.chat.type == "private")
async def creation_title(message: Message, state: FSMContext) -> None:
    title = message.text.strip()
    if not title:
        await message.answer(MESSAGES["ask_title"])
        return
    await state.update_data(title=title)
    await message.answer(MESSAGES["ask_deadline"])
    await state.set_state(MeetingCreation.waiting_deadline)


# ─── СОЗДАНИЕ: дедлайн ────────────────────────────────────────────────────────

@router.message(MeetingCreation.waiting_deadline, F.chat.type == "private")
async def creation_deadline(message: Message, state: FSMContext, bot: Bot) -> None:
    deadline_dt = parse_deadline(message.text)
    if deadline_dt is None:
        await message.answer(MESSAGES["invalid_deadline"])
        return
    if deadline_dt <= now_moscow():
        await message.answer(MESSAGES["deadline_in_past"])
        return

    data          = await state.get_data()
    title         = data["title"]
    group_chat_id = data["group_chat_id"]
    created_by    = data["created_by"]

    meeting_id = await create_meeting(
        title=title,
        group_chat_id=group_chat_id,
        created_by=created_by,
        deadline=deadline_dt.isoformat(),
        created_at=now_moscow().isoformat(),
    )

    bot_info         = await bot.get_me()
    deadline_display = format_datetime_moscow(deadline_dt)
    announce_text    = MESSAGES["announce_template"].format(
        title=title,
        deadline=deadline_display,
        count=0,
    )
    kb   = build_announce_keyboard(meeting_id, bot_info.username)
    sent = await bot.send_message(
        chat_id=group_chat_id,
        text=announce_text,
        reply_markup=kb,
    )
    await save_message_id(meeting_id, sent.message_id)

    await state.clear()
    await message.answer(MESSAGES["meeting_created"])
    logger.info(f"Встреча #{meeting_id} '{title}' создана в чате {group_chat_id}.")


# ─── ГОЛОСОВАНИЕ: календарь ───────────────────────────────────────────────────

@router.callback_query(CalendarCallback.filter(), VoteFlow.choosing_dates)
async def vote_calendar(
    callback: CallbackQuery,
    callback_data: CalendarCallback,
    state: FSMContext,
) -> None:
    data           = await state.get_data()
    selected_dates = set(data.get("selected_dates", []))
    meeting_id     = data.get("meeting_id")
    meeting        = await get_meeting(meeting_id)
    deadline_dt    = datetime.fromisoformat(meeting["deadline"])

    action = callback_data.action
    year   = callback_data.year
    month  = callback_data.month

    if action == "ignore":
        await callback.answer()
        return

    if action == "prev":
        month, year = (12, year - 1) if month == 1 else (month - 1, year)
        await callback.message.edit_reply_markup(
            reply_markup=build_calendar(year, month, selected_dates)
        )
        await callback.answer()
        return

    if action == "next":
        month, year = (1, year + 1) if month == 12 else (month + 1, year)
        await callback.message.edit_reply_markup(
            reply_markup=build_calendar(year, month, selected_dates)
        )
        await callback.answer()
        return

    if action == "day":
        date_str  = f"{year:04d}-{month:02d}-{callback_data.day:02d}"
        chosen_dt = datetime.strptime(date_str, "%Y-%m-%d")
        if chosen_dt.date() > deadline_dt.date():
            await callback.answer("⛔ Эта дата позже дедлайна.", show_alert=True)
            return
        if date_str in selected_dates:
            selected_dates.discard(date_str)
        else:
            selected_dates.add(date_str)
        await state.update_data(selected_dates=list(selected_dates))
        await callback.message.edit_reply_markup(
            reply_markup=build_calendar(year, month, selected_dates)
        )
        await callback.answer()
        return

    if action == "done":
        if not selected_dates:
            await callback.answer(MESSAGES["no_dates_selected"], show_alert=True)
            return

        sorted_dates = sorted(selected_dates)

        # Восстанавливаем slots_dict из предыдущего сеанса (если был)
        slots_dict = _slots_dict_from_state(data.get("slots_dict", {}))

        # Убираем даты, которые больше не выбраны
        slots_dict = {k: v for k, v in slots_dict.items() if k in set(sorted_dates)}

        await state.update_data(
            selected_dates=sorted_dates,
            slots_dict=_slots_dict_to_state(slots_dict),
            slot_mode="per_date",
            current_date_index=0,
            same_time_dates=[],
            same_slots=[],
        )
        kb = build_slots_keyboard(
            dates=sorted_dates,
            selected_slots=slots_dict,
            mode="per_date",
            current_date_index=0,
            same_time_dates=set(),
        )
        await callback.message.edit_text(MESSAGES["ask_slots_mode"], reply_markup=kb)
        await state.set_state(VoteFlow.choosing_slots)
        await callback.answer()


# ─── ГОЛОСОВАНИЕ: SlotActionCallback ─────────────────────────────────────────

@router.callback_query(SlotActionCallback.filter(), VoteFlow.choosing_slots)
async def vote_slot_action(
    callback: CallbackQuery,
    callback_data: SlotActionCallback,
    state: FSMContext,
    bot: Bot,
) -> None:
    data               = await state.get_data()
    slot_mode          = data.get("slot_mode", "per_date")
    sorted_dates       = data.get("selected_dates", [])
    current_date_index = data.get("current_date_index", 0)
    slots_dict         = _slots_dict_from_state(data.get("slots_dict", {}))
    same_time_dates    = set(data.get("same_time_dates", []))
    same_slots         = set(data.get("same_slots", []))
    meeting_id         = data.get("meeting_id")

    action = callback_data.action

    # ── noop ──────────────────────────────────────────────────────────────────
    if action == "noop":
        await callback.answer()
        return

    # ── Переключение режима ───────────────────────────────────────────────────
    if action == "mode":
        slot_mode = "same" if slot_mode == "per_date" else "per_date"
        # Сбрасываем только временное состояние same, основные слоты сохраняем
        await state.update_data(
            slot_mode=slot_mode,
            current_date_index=0,
            same_time_dates=[],
            same_slots=[],
        )
        await callback.message.edit_reply_markup(
            reply_markup=build_slots_keyboard(
                dates=sorted_dates,
                selected_slots=slots_dict,
                mode=slot_mode,
                current_date_index=0,
                same_time_dates=set(),
            )
        )
        await callback.answer()
        return

    # ── Навигация prev / next (per_date) ──────────────────────────────────────
    if action in ("prev", "next"):
        new_index = callback_data.value
        await state.update_data(current_date_index=new_index)
        await callback.message.edit_reply_markup(
            reply_markup=build_slots_keyboard(
                dates=sorted_dates,
                selected_slots=slots_dict,
                mode=slot_mode,
                current_date_index=new_index,
                same_time_dates=same_time_dates,
            )
        )
        await callback.answer()
        return

    # ── Toggle даты в режиме same ─────────────────────────────────────────────
    if action == "toggle_date":
        i = callback_data.value
        if i in same_time_dates:
            same_time_dates.discard(i)
        else:
            same_time_dates.add(i)
        await state.update_data(same_time_dates=list(same_time_dates))
        await callback.message.edit_reply_markup(
            reply_markup=build_slots_keyboard(
                dates=sorted_dates,
                selected_slots={**slots_dict, "__same__": same_slots},
                mode=slot_mode,
                current_date_index=current_date_index,
                same_time_dates=same_time_dates,
            )
        )
        await callback.answer()
        return

    # ── Применить одинаковое время к отмеченным датам ────────────────────────
    if action == "apply_same":
        if not same_time_dates:
            await callback.answer("⚠️ Не выбрано ни одной даты.", show_alert=True)
            return
        if not same_slots:
            await callback.answer("⚠️ Не выбрано ни одного времени.", show_alert=True)
            return

        for i in same_time_dates:
            slots_dict[sorted_dates[i]] = set(same_slots)

        await state.update_data(
            slots_dict=_slots_dict_to_state(slots_dict),
            same_time_dates=[],
            same_slots=[],
        )
        await callback.message.edit_reply_markup(
            reply_markup=build_slots_keyboard(
                dates=sorted_dates,
                selected_slots=slots_dict,
                mode=slot_mode,
                current_date_index=current_date_index,
                same_time_dates=set(),
            )
        )
        await callback.answer("✅ Время применено!")
        return

    # ── Готово ────────────────────────────────────────────────────────────────
    if action == "done":
        final_slots = sorted(
            f"{date_str} {time_str}"
            for date_str, times in slots_dict.items()
            for time_str in times
            if date_str in set(sorted_dates)
        )

        if not final_slots:
            await callback.answer(MESSAGES["no_slots_selected"], show_alert=True)
            return

        user_id  = callback.from_user.id
        user     = callback.from_user
        username = f"@{user.username}" if user.username else user.full_name

        await delete_user_slots(meeting_id, user_id)
        for slot_str in final_slots:
            await add_user_slot(meeting_id, user_id, slot_str)
        await upsert_participant(meeting_id, user_id, username)

        # Обновляем счётчик в анонсе группы
        meeting = await get_meeting(meeting_id)
        count   = await count_participants(meeting_id)
        if meeting and meeting.get("message_id"):
            deadline_dt      = datetime.fromisoformat(meeting["deadline"])
            deadline_display = format_datetime_moscow(deadline_dt)
            announce_text    = MESSAGES["announce_template"].format(
                title=meeting["title"],
                deadline=deadline_display,
                count=count,
            )
            bot_info    = await bot.get_me()
            kb_announce = build_announce_keyboard(meeting_id, bot_info.username)
            try:
                await bot.edit_message_text(
                    chat_id=meeting["group_chat_id"],
                    message_id=meeting["message_id"],
                    text=announce_text,
                    reply_markup=kb_announce,
                )
            except Exception as e:
                logger.warning(f"Не удалось обновить анонс встречи #{meeting_id}: {e}")

        await callback.message.edit_text(MESSAGES["vote_saved"])
        await state.clear()
        await callback.answer()
        logger.info(f"Пользователь {user_id} сохранил слоты для встречи #{meeting_id}.")


# ─── ГОЛОСОВАНИЕ: toggle слота ────────────────────────────────────────────────

@router.callback_query(SlotCallback.filter(), VoteFlow.choosing_slots)
async def vote_slot_toggle(
    callback: CallbackQuery,
    callback_data: SlotCallback,
    state: FSMContext,
) -> None:
    data               = await state.get_data()
    slot_mode          = data.get("slot_mode", "per_date")
    sorted_dates       = data.get("selected_dates", [])
    current_date_index = data.get("current_date_index", 0)
    slots_dict         = _slots_dict_from_state(data.get("slots_dict", {}))
    same_time_dates    = set(data.get("same_time_dates", []))
    same_slots         = set(data.get("same_slots", []))

    time_str = decode_slot_key(callback_data.slot_key)  # всегда HH:MM

    if slot_mode == "per_date":
        date_str       = sorted_dates[current_date_index]
        slots_for_date = slots_dict.get(date_str, set())
        if time_str in slots_for_date:
            slots_for_date.discard(time_str)
        else:
            slots_for_date.add(time_str)
        slots_dict[date_str] = slots_for_date
        await state.update_data(slots_dict=_slots_dict_to_state(slots_dict))

        await callback.message.edit_reply_markup(
            reply_markup=build_slots_keyboard(
                dates=sorted_dates,
                selected_slots=slots_dict,
                mode=slot_mode,
                current_date_index=current_date_index,
                same_time_dates=same_time_dates,
            )
        )

    else:  # same
        if time_str in same_slots:
            same_slots.discard(time_str)
        else:
            same_slots.add(time_str)
        await state.update_data(same_slots=list(same_slots))

        await callback.message.edit_reply_markup(
            reply_markup=build_slots_keyboard(
                dates=sorted_dates,
                selected_slots={**slots_dict, "__same__": same_slots},
                mode=slot_mode,
                current_date_index=current_date_index,
                same_time_dates=same_time_dates,
            )
        )

    await callback.answer()
