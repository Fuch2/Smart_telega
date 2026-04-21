import sys
import asyncio
import logging

if sys.platform == "darwin":
    asyncio.set_event_loop_policy(asyncio.DefaultEventLoopPolicy())

from aiogram import Bot, Dispatcher
from aiogram.fsm.storage.memory import MemoryStorage
from aiogram.client.default import DefaultBotProperties
from aiogram.enums import ParseMode
from aiogram.types import (
    BotCommand,
    BotCommandScopeAllGroupChats,
    BotCommandScopeAllPrivateChats,
    MenuButtonCommands,
)

from config import BOT_TOKEN
from database import init_db
from handlers import group, private
from scheduler import scheduler_loop

# ─── ЛОГИРОВАНИЕ ─────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)


# ─── КОМАНДЫ И МЕНЮ ──────────────────────────────────────────────────────────

async def set_bot_commands(bot: Bot) -> None:
    # Команды в группах — видны в меню слева от поля ввода
    await bot.set_my_commands(
        commands=[
            BotCommand(command="newmeeting", description="📅 Создать встречу"),
            BotCommand(command="meetings",   description="📋 Список активных встреч"),
            BotCommand(command="results",    description="📊 Показать результаты"),
            BotCommand(command="cancel",     description="❌ Отменить встречу"),
        ],
        scope=BotCommandScopeAllGroupChats(),
    )

    # Команды в личке
    await bot.set_my_commands(
        commands=[
            BotCommand(command="start", description="👋 Начать"),
            BotCommand(command="calendar", description="📅 Мой календарь"),
            BotCommand(command="integrations", description="🔗 Интеграции"),
        ],
        scope=BotCommandScopeAllPrivateChats(),
    )

    # Включаем кнопку «Меню» (значок ☰) глобально
    await bot.set_chat_menu_button(
        menu_button=MenuButtonCommands(),
    )

    logger.info("Команды бота и кнопка меню установлены.")


# ─── ТОЧКА ВХОДА ─────────────────────────────────────────────────────────────

async def main() -> None:
    logger.info("Инициализация базы данных...")
    await init_db()

    bot = Bot(
        token=BOT_TOKEN,
        default=DefaultBotProperties(parse_mode=ParseMode.HTML),
    )
    dp = Dispatcher(storage=MemoryStorage())

    dp.include_router(group.router)
    dp.include_router(private.router)

    await set_bot_commands(bot)

    scheduler_task = asyncio.create_task(scheduler_loop(bot))
    logger.info("Планировщик запущен как фоновая задача.")

    logger.info("Бот запускается в режиме polling...")
    try:
        await dp.start_polling(bot, allowed_updates=dp.resolve_used_update_types())
    finally:
        scheduler_task.cancel()
        try:
            await scheduler_task
        except asyncio.CancelledError:
            pass
        await bot.session.close()
        logger.info("Бот остановлен.")


if __name__ == "__main__":
    asyncio.run(main())
