import logging
from datetime import datetime

import pytz
from aiogram import Router, F, Bot
from aiogram.filters import Command, CommandStart
from aiogram.fsm.context import FSMContext
from aiogram.fsm.state import State, StatesGroup
from aiogram.types import Message, CallbackQuery

from config import TIMEZONE
from database import (
    get_meeting,
    get_meeting_custom_slots,
    get_user_active_meetings,
    get_user_integrations,
    remember_meeting_access,
    add_user_slot,
    add_meeting_custom_slot,
    delete_user_slots,
    get_user_slots,
    get_participant_votes,
    upsert_participant,
    count_participants,
    save_message_id,
    create_meeting,
)
from keyboards.inline import (
    build_calendar,
    build_slots_keyboard,
    build_available_slots_by_date,
    build_announce_keyboard,
    build_private_home_keyboard,
    build_personal_calendar_keyboard,
    build_integrations_keyboard,
    CalendarCallback,
    SlotCallback,
    SlotActionCallback,
    PrivateMenuCallback,
    decode_slot_key,
)
from utils import (
    MESSAGES,
    now_moscow,
    format_datetime_moscow,
    format_participant_votes,
    parse_deadline,
    parse_time_input,
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
    waiting_custom_time = State()


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


def _meeting_relation_label(meeting: dict) -> str:
    tags = []
    if meeting.get("is_creator"):
        tags.append("организатор")
    if meeting.get("has_voted"):
        tags.append("голос сохранён")
    elif meeting.get("has_opened"):
        tags.append("ещё не голосовал")
    return ", ".join(tags) if tags else "ещё не голосовал"


def _integration_status_map(integrations: list[dict]) -> dict[str, str]:
    providers = {item["provider"] for item in integrations}
    return {
        "google": MESSAGES["integration_connected"] if "google" in providers else MESSAGES["integration_not_connected"],
        "todoist": MESSAGES["integration_connected"] if "todoist" in providers else MESSAGES["integration_not_connected"],
    }


async def _get_chat_titles(bot: Bot, meetings: list[dict]) -> dict[int, str]:
    titles: dict[int, str] = {}
    for meeting in meetings:
        chat_id = meeting["group_chat_id"]
        if chat_id in titles:
            continue
        try:
            chat = await bot.get_chat(chat_id)
            titles[chat_id] = getattr(chat, "title", None) or str(chat_id)
        except Exception:
            titles[chat_id] = str(chat_id)
    return titles


async def _send_personal_calendar(message: Message, bot: Bot, user_id: int) -> None:
    meetings = await get_user_active_meetings(user_id)
    if not meetings:
        await message.answer(MESSAGES["personal_calendar_empty"])
        return

    chat_titles = await _get_chat_titles(bot, meetings)
    lines = []
    for meeting in meetings:
        deadline_dt = datetime.fromisoformat(meeting["deadline"])
        source = chat_titles.get(meeting["group_chat_id"], str(meeting["group_chat_id"]))
        status = _meeting_relation_label(meeting)
        lines.append(
            "\n".join([
                f"• <b>{meeting['title']}</b>",
                f"Источник: {source}",
                f"Дедлайн: {format_datetime_moscow(deadline_dt)}",
                f"Статус: {status}",
            ])
        )

    bot_info = await bot.get_me()
    await message.answer(
        MESSAGES["personal_calendar_header"].format(lines="\n\n".join(lines)),
        reply_markup=build_personal_calendar_keyboard(meetings, bot_info.username, chat_titles),
    )


async def _send_private_home(message: Message) -> None:
    await message.answer(
        MESSAGES["private_welcome"],
        reply_markup=build_private_home_keyboard(),
    )


async def _send_integrations(message: Message, user_id: int) -> None:
    integrations = await get_user_integrations(user_id)
    statuses = _integration_status_map(integrations)
    await message.answer(
        MESSAGES["integrations_header"].format(
            google_status=statuses["google"],
            todoist_status=statuses["todoist"],
        ),
        reply_markup=build_integrations_keyboard(),
    )


async def _build_slots_reply_markup(
    meeting_id: int,
    dates: list[str],
    slots_dict: dict[str, set[str]],
    slot_mode: str,
    current_date_index: int,
    same_time_dates: set[int],
    same_slots: set[str],
):
    custom_slots = await get_meeting_custom_slots(meeting_id)
    available_slots_by_date = build_available_slots_by_date(dates, custom_slots)
    selected_slots = dict(slots_dict)
    selected_slots["__same__"] = set(same_slots)
    return build_slots_keyboard(
        dates=dates,
        selected_slots=selected_slots,
        mode=slot_mode,
        current_date_index=current_date_index,
        same_time_dates=same_time_dates,
        available_slots_by_date=available_slots_by_date,
    )


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

        await remember_meeting_access(
            meeting_id=meeting_id,
            user_id=message.from_user.id,
            accessed_at=now_moscow().isoformat(),
        )

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
            custom_slot_targets=[],
        )

        deadline_display = format_datetime_moscow(deadline_dt)
        await message.answer(
            MESSAGES["vote_intro"].format(
                title=meeting["title"],
                deadline=deadline_display,
            )
        )
        votes = await get_participant_votes(meeting_id)
        await message.answer(format_participant_votes(votes))

        now = now_moscow()
        kb  = build_calendar(now.year, now.month, set(saved_dates))
        await message.answer(MESSAGES["ask_dates"], reply_markup=kb)
        await state.set_state(VoteFlow.choosing_dates)
        return

    # ── /start без payload ────────────────────────────────────────────────────
    await state.clear()
    await _send_private_home(message)


@router.message(Command("calendar"), F.chat.type == "private")
async def cmd_calendar_private(message: Message, bot: Bot) -> None:
    await _send_personal_calendar(message, bot, message.from_user.id)


@router.message(Command("integrations"), F.chat.type == "private")
async def cmd_integrations_private(message: Message) -> None:
    await _send_integrations(message, message.from_user.id)


@router.callback_query(PrivateMenuCallback.filter(F.action == "calendar"), F.message.chat.type == "private")
async def private_menu_calendar(callback: CallbackQuery, bot: Bot) -> None:
    await _send_personal_calendar(callback.message, bot, callback.from_user.id)
    await callback.answer()


@router.callback_query(PrivateMenuCallback.filter(F.action == "integrations"), F.message.chat.type == "private")
async def private_menu_integrations(callback: CallbackQuery) -> None:
    await _send_integrations(callback.message, callback.from_user.id)
    await callback.answer()


@router.callback_query(PrivateMenuCallback.filter(F.action == "home"), F.message.chat.type == "private")
async def private_menu_home(callback: CallbackQuery) -> None:
    await _send_private_home(callback.message)
    await callback.answer()


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
            custom_slot_targets=[],
        )
        kb = await _build_slots_reply_markup(
            meeting_id=meeting_id,
            dates=sorted_dates,
            slots_dict=slots_dict,
            slot_mode="per_date",
            current_date_index=0,
            same_time_dates=set(),
            same_slots=set(),
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
        if slot_mode == "same" and same_time_dates and same_slots:
            for i in same_time_dates:
                slots_dict[sorted_dates[i]] = set(same_slots)
        slot_mode = "same" if slot_mode == "per_date" else "per_date"
        await state.update_data(
            slots_dict=_slots_dict_to_state(slots_dict),
            slot_mode=slot_mode,
            current_date_index=0,
            same_time_dates=[],
            same_slots=[],
        )
        await callback.message.edit_reply_markup(
            reply_markup=await _build_slots_reply_markup(
                meeting_id=meeting_id,
                dates=sorted_dates,
                slots_dict=slots_dict,
                slot_mode=slot_mode,
                current_date_index=0,
                same_time_dates=set(),
                same_slots=set(),
            )
        )
        await callback.answer()
        return

    # ── Навигация prev / next (per_date) ──────────────────────────────────────
    if action in ("prev", "next"):
        new_index = callback_data.value
        await state.update_data(current_date_index=new_index)
        await callback.message.edit_reply_markup(
            reply_markup=await _build_slots_reply_markup(
                meeting_id=meeting_id,
                dates=sorted_dates,
                slots_dict=slots_dict,
                slot_mode=slot_mode,
                current_date_index=new_index,
                same_time_dates=same_time_dates,
                same_slots=same_slots,
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
            reply_markup=await _build_slots_reply_markup(
                meeting_id=meeting_id,
                dates=sorted_dates,
                slots_dict=slots_dict,
                slot_mode=slot_mode,
                current_date_index=current_date_index,
                same_time_dates=same_time_dates,
                same_slots=same_slots,
            )
        )
        await callback.answer()
        return

    # ── Добавить своё время ───────────────────────────────────────────────────
    if action == "add_custom":
        if slot_mode == "per_date":
            target_dates = [sorted_dates[current_date_index]]
            target_label = datetime.strptime(target_dates[0], "%Y-%m-%d").strftime("%d.%m.%Y")
            prompt = MESSAGES["ask_custom_time_single"].format(date=target_label)
        else:
            if not same_time_dates:
                await callback.answer(MESSAGES["custom_time_no_dates"], show_alert=True)
                return
            target_dates = [sorted_dates[i] for i in sorted(same_time_dates)]
            dates_label = ", ".join(
                datetime.strptime(date_str, "%Y-%m-%d").strftime("%d.%m")
                for date_str in target_dates
            )
            prompt = MESSAGES["ask_custom_time_multiple"].format(dates=dates_label)

        await state.update_data(custom_slot_targets=target_dates)
        await state.set_state(VoteFlow.waiting_custom_time)
        await callback.message.answer(prompt)
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
            reply_markup=await _build_slots_reply_markup(
                meeting_id=meeting_id,
                dates=sorted_dates,
                slots_dict=slots_dict,
                slot_mode=slot_mode,
                current_date_index=current_date_index,
                same_time_dates=set(),
                same_slots=set(),
            )
        )
        await callback.answer("✅ Время применено!")
        return

    # ── Готово ────────────────────────────────────────────────────────────────
    if action == "done":
        if slot_mode == "same" and same_time_dates and same_slots:
            for i in same_time_dates:
                slots_dict[sorted_dates[i]] = set(same_slots)

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
        votes = await get_participant_votes(meeting_id)
        await callback.message.answer(format_participant_votes(votes))
        await state.clear()
        await callback.answer()
        logger.info(f"Пользователь {user_id} сохранил слоты для встречи #{meeting_id}.")


@router.message(VoteFlow.waiting_custom_time, F.chat.type == "private")
async def vote_custom_time_input(message: Message, state: FSMContext) -> None:
    time_str = parse_time_input(message.text)
    if time_str is None:
        await message.answer(MESSAGES["invalid_custom_time"])
        return

    data               = await state.get_data()
    meeting_id         = data.get("meeting_id")
    sorted_dates       = data.get("selected_dates", [])
    slot_mode          = data.get("slot_mode", "per_date")
    current_date_index = data.get("current_date_index", 0)
    slots_dict         = _slots_dict_from_state(data.get("slots_dict", {}))
    same_time_dates    = set(data.get("same_time_dates", []))
    same_slots         = set(data.get("same_slots", []))
    target_dates       = data.get("custom_slot_targets", [])

    if not target_dates:
        if slot_mode == "per_date" and sorted_dates:
            target_dates = [sorted_dates[current_date_index]]
        elif slot_mode == "same" and same_time_dates:
            target_dates = [sorted_dates[i] for i in sorted(same_time_dates)]

    if not target_dates:
        await state.set_state(VoteFlow.choosing_slots)
        await message.answer(MESSAGES["custom_time_no_dates"])
        return

    for date_str in target_dates:
        await add_meeting_custom_slot(
            meeting_id=meeting_id,
            slot_datetime=f"{date_str} {time_str}",
            created_by=message.from_user.id,
        )

    if slot_mode == "per_date":
        date_str = target_dates[0]
        slots_for_date = slots_dict.get(date_str, set())
        slots_for_date.add(time_str)
        slots_dict[date_str] = slots_for_date
    else:
        same_slots.add(time_str)

    await state.update_data(
        slots_dict=_slots_dict_to_state(slots_dict),
        same_slots=list(same_slots),
        custom_slot_targets=[],
    )
    await state.set_state(VoteFlow.choosing_slots)

    reply_markup = await _build_slots_reply_markup(
        meeting_id=meeting_id,
        dates=sorted_dates,
        slots_dict=slots_dict,
        slot_mode=slot_mode,
        current_date_index=current_date_index,
        same_time_dates=same_time_dates,
        same_slots=same_slots,
    )
    await message.answer(
        MESSAGES["custom_time_added"].format(time=time_str),
        reply_markup=reply_markup,
    )


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
            reply_markup=await _build_slots_reply_markup(
                meeting_id=meeting_id,
                dates=sorted_dates,
                slots_dict=slots_dict,
                slot_mode=slot_mode,
                current_date_index=current_date_index,
                same_time_dates=same_time_dates,
                same_slots=same_slots,
            )
        )

    else:  # same
        if time_str in same_slots:
            same_slots.discard(time_str)
        else:
            same_slots.add(time_str)
        await state.update_data(same_slots=list(same_slots))

        await callback.message.edit_reply_markup(
            reply_markup=await _build_slots_reply_markup(
                meeting_id=meeting_id,
                dates=sorted_dates,
                slots_dict=slots_dict,
                slot_mode=slot_mode,
                current_date_index=current_date_index,
                same_time_dates=same_time_dates,
                same_slots=same_slots,
            )
        )

    await callback.answer()
