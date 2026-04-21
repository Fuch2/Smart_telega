import logging
from datetime import datetime

import pytz
from aiogram import Router, F, Bot
from aiogram.filters import Command, ChatMemberUpdatedFilter, JOIN_TRANSITION
from aiogram.types import (
    Message,
    ChatMemberUpdated,
    InlineKeyboardButton,
    ChatMemberAdministrator,
    ChatMemberOwner,
    MenuButtonCommands,
    BotCommand,
)
from aiogram.types import BotCommandScopeChat
from aiogram.utils.keyboard import InlineKeyboardBuilder

from config import TIMEZONE
from database import (
    get_active_meetings_by_group,
    cancel_meeting,
    get_common_slots,
    get_participants,
)
from keyboards.inline import (
    build_meetings_list_keyboard,
)
from utils import (
    MESSAGES,
    format_datetime_moscow,
    format_participant_names,
)

logger = logging.getLogger(__name__)
router = Router()
TZ = pytz.timezone(TIMEZONE)

GROUP_COMMANDS = [
    BotCommand(command="newmeeting", description="📅 Создать встречу"),
    BotCommand(command="meetings",   description="📋 Список встреч"),
    BotCommand(command="results",    description="📊 Результаты"),
    BotCommand(command="cancel",     description="❌ Отменить встречу"),
]


# ─── Бот добавлен в группу ────────────────────────────────────────────────────

@router.my_chat_member(
    ChatMemberUpdatedFilter(member_status_changed=JOIN_TRANSITION),
    F.chat.type.in_({"group", "supergroup"}),
)
async def bot_added_to_group(event: ChatMemberUpdated, bot: Bot) -> None:
    chat_id = event.chat.id
    try:
        await bot.set_chat_menu_button(
            chat_id=chat_id,
            menu_button=MenuButtonCommands(),
        )
        await bot.set_my_commands(
            commands=GROUP_COMMANDS,
            scope=BotCommandScopeChat(chat_id=chat_id),
        )
        logger.info(f"Меню установлено для чата {chat_id}.")
    except Exception as e:
        logger.warning(f"Не удалось установить меню для чата {chat_id}: {e}")


# ─── /newmeeting ─────────────────────────────────────────────────────────────

@router.message(Command("newmeeting"), F.chat.type.in_({"group", "supergroup"}))
async def cmd_newmeeting(message: Message, bot: Bot) -> None:
    bot_info = await bot.get_me()
    chat_id  = message.chat.id

    # Удаляем команду пользователя сразу
    try:
        await bot.delete_message(chat_id=chat_id, message_id=message.message_id)
    except Exception:
        pass

    # Шлём приглашение без кнопки — нам нужен message_id до того, как вставим его в ссылку
    sent = await bot.send_message(
        chat_id=chat_id,
        text="📅 Нажми кнопку ниже — я задам несколько вопросов в личке, "
             "и встреча появится здесь.",
    )
    invite_msg_id = sent.message_id

    # Редактируем, вставляя реальный message_id в deep link
    builder = InlineKeyboardBuilder()
    builder.row(InlineKeyboardButton(
        text="✏️ Создать встречу",
        url=f"https://t.me/{bot_info.username}?start=newmeeting_{chat_id}_{invite_msg_id}",
    ))
    await bot.edit_message_reply_markup(
        chat_id=chat_id,
        message_id=invite_msg_id,
        reply_markup=builder.as_markup(),
    )


# ─── /meetings ───────────────────────────────────────────────────────────────

@router.message(Command("meetings"), F.chat.type.in_({"group", "supergroup"}))
async def cmd_meetings(message: Message, bot: Bot) -> None:
    meetings = await get_active_meetings_by_group(message.chat.id)
    if not meetings:
        await message.answer(MESSAGES["no_active_meetings"])
        return

    bot_info = await bot.get_me()
    kb       = build_meetings_list_keyboard(meetings, bot_info.username)
    lines    = ["📋 Активные встречи в этом чате:\n"]
    for m in meetings:
        deadline_dt = datetime.fromisoformat(m["deadline"])
        lines.append(f"• {m['title']} (дедлайн: {format_datetime_moscow(deadline_dt)})")
    await message.answer("\n".join(lines), reply_markup=kb)


# ─── /results ────────────────────────────────────────────────────────────────

@router.message(Command("results"), F.chat.type.in_({"group", "supergroup"}))
async def cmd_results(message: Message, bot: Bot) -> None:
    meetings = await get_active_meetings_by_group(message.chat.id)
    if not meetings:
        await message.answer(MESSAGES["no_active_meetings"])
        return

    await _send_results(message.chat.id, meetings[0], bot)


async def _send_results(chat_id: int, meeting: dict, bot: Bot) -> None:
    meeting_id       = meeting["id"]
    common           = await get_common_slots(meeting_id)
    participants     = await get_participants(meeting_id)
    total            = len(participants)
    participants_txt = format_participant_names(participants)

    if total == 0:
        text = MESSAGES["results_no_participants"].format(title=meeting["title"])
    elif not common:
        text = MESSAGES["results_no_common"].format(
            title=meeting["title"],
            total=total,
            participants=participants_txt,
        )
    else:
        slots_lines = "\n".join(
            f"  • {format_datetime_moscow(datetime.strptime(s, '%Y-%m-%d %H:%M'))}"
            for s in common
        )
        text = MESSAGES["results_found"].format(
            title=meeting["title"],
            total=total,
            participants=participants_txt,
            slots=slots_lines,
        )

    await bot.send_message(chat_id=chat_id, text=text)
    logger.info(f"Результаты встречи #{meeting_id} отправлены в чат {chat_id}.")


# ─── /cancel ─────────────────────────────────────────────────────────────────

@router.message(Command("cancel"), F.chat.type.in_({"group", "supergroup"}))
async def cmd_cancel(message: Message, bot: Bot) -> None:
    meetings = await get_active_meetings_by_group(message.chat.id)
    if not meetings:
        await message.answer(MESSAGES["cancel_no_active"])
        return

    user_id = message.from_user.id

    async def is_admin(uid: int) -> bool:
        member = await bot.get_chat_member(message.chat.id, uid)
        return isinstance(member, (ChatMemberAdministrator, ChatMemberOwner))

    admin = await is_admin(user_id)

    if len(meetings) == 1:
        meeting = meetings[0]
        if meeting["created_by"] != user_id and not admin:
            await message.answer(MESSAGES["cancel_no_permission"])
            return
        await cancel_meeting(meeting["id"])
        await message.answer(
            MESSAGES["meeting_cancelled"].format(title=meeting["title"])
        )
        logger.info(f"Встреча #{meeting['id']} отменена пользователем {user_id}.")
        return

    for meeting in meetings:
        if meeting["created_by"] == user_id or admin:
            await cancel_meeting(meeting["id"])
            await message.answer(
                MESSAGES["meeting_cancelled"].format(title=meeting["title"])
            )
            logger.info(f"Встреча #{meeting['id']} отменена пользователем {user_id}.")
            return

    await message.answer(MESSAGES["cancel_no_permission"])
