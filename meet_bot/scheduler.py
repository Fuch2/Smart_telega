import asyncio
import logging
from datetime import datetime

from aiogram import Bot
import pytz

from config import TIMEZONE, SCHEDULER_INTERVAL
from database import (
    get_all_active_meetings,
    close_meeting,
    get_participants,
    get_common_slots,          # ← было: get_slots (не существует)
)
from utils import (
    MESSAGES,
    now_moscow,
    slot_to_display,
)
from keyboards.inline import build_announce_keyboard

logger = logging.getLogger(__name__)
TZ = pytz.timezone(TIMEZONE)


async def check_deadlines(bot: Bot) -> None:
    """Проверяет дедлайны всех активных встреч и закрывает просроченные."""
    meetings = await get_all_active_meetings()
    now = now_moscow()

    for meeting in meetings:
        try:
            deadline_dt = datetime.fromisoformat(meeting["deadline"])
            if deadline_dt.tzinfo is None:
                deadline_dt = TZ.localize(deadline_dt)

            if now >= deadline_dt:
                logger.info(
                    f"Дедлайн встречи #{meeting['id']} '{meeting['title']}' истёк. Закрываю."
                )
                await close_meeting(meeting["id"])
                await publish_results(bot, meeting)

        except Exception as e:
            logger.error(
                f"Ошибка при обработке встречи #{meeting['id']}: {e}", exc_info=True
            )


async def publish_results(bot: Bot, meeting: dict) -> None:
    """Публикует итоги встречи в групповой чат."""
    meeting_id = meeting["id"]
    title = meeting["title"]
    group_chat_id = meeting["group_chat_id"]
    message_id = meeting.get("message_id")

    common_slots = await get_common_slots(meeting_id)        # ← было: find_common_slots
    participants = await get_participants(meeting_id)
    participant_names = (
        ", ".join(p["username"] for p in participants) if participants else "—"
    )

    if common_slots:
        slots_lines = "\n".join(
            f"• {slot_to_display(s)}" for s in common_slots  # ← get_common_slots возвращает list[str]
        )
        result_text = MESSAGES["results_found"].format(       # ← было: "result_found"
            title=title,
            total=len(participants),                          # ← шаблон требует {total}, не {participants}
            slots=slots_lines,
        )
    else:
        result_text = MESSAGES["results_no_common"].format(   # ← было: "result_not_found"
            title=title,
            total=len(participants),
        )

    try:
        if message_id:
            await bot.send_message(
                chat_id=group_chat_id,
                text=result_text,
                reply_to_message_id=message_id,
            )
        else:
            await bot.send_message(chat_id=group_chat_id, text=result_text)
        logger.info(f"Итоги встречи #{meeting_id} опубликованы в чат {group_chat_id}.")
    except Exception as e:
        logger.error(
            f"Не удалось опубликовать итоги встречи #{meeting_id}: {e}", exc_info=True
        )


async def scheduler_loop(bot: Bot) -> None:
    """Бесконечный цикл планировщика."""
    logger.info("Планировщик запущен.")
    while True:
        try:
            await check_deadlines(bot)
        except Exception as e:
            logger.error(f"Ошибка в планировщике: {e}", exc_info=True)
        await asyncio.sleep(SCHEDULER_INTERVAL)
