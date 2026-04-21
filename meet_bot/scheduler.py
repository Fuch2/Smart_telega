import asyncio
import logging
from datetime import datetime

from aiogram import Bot
import pytz

from config import TIMEZONE, SCHEDULER_INTERVAL
from database import (
    get_all_active_meetings,
    finalize_meeting,
    get_participants,
    get_common_slots,
)
from utils import (
    MESSAGES,
    now_moscow,
    format_participant_names,
    slot_to_display,
)

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
                common_slots = await get_common_slots(meeting["id"])
                final_slot = common_slots[0] if common_slots else None
                await finalize_meeting(
                    meeting_id=meeting["id"],
                    finalized_at=now.isoformat(),
                    final_slot_datetime=final_slot,
                )
                await publish_results(bot, meeting, final_slot)

        except Exception as e:
            logger.error(
                f"Ошибка при обработке встречи #{meeting['id']}: {e}", exc_info=True
            )


async def publish_results(bot: Bot, meeting: dict, final_slot: str | None = None) -> None:
    """Публикует итоги встречи в групповой чат."""
    meeting_id = meeting["id"]
    title = meeting["title"]
    group_chat_id = meeting["group_chat_id"]
    message_id = meeting.get("message_id")

    participants = await get_participants(meeting_id)
    participant_names = format_participant_names(participants)

    if not participants:
        result_text = MESSAGES["results_no_participants"].format(title=title)
    elif final_slot:
        result_text = MESSAGES["results_confirmed"].format(
            title=title,
            total=len(participants),
            participants=participant_names,
            slot=slot_to_display(final_slot),
        )
    else:
        result_text = MESSAGES["results_no_common"].format(
            title=title,
            total=len(participants),
            participants=participant_names,
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
