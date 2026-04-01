import aiosqlite
import logging
from typing import Optional
from config import DB_PATH

logger = logging.getLogger(__name__)


async def init_db() -> None:
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute("""
            CREATE TABLE IF NOT EXISTS meetings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                title TEXT NOT NULL,
                group_chat_id INTEGER NOT NULL,
                created_by INTEGER NOT NULL,
                deadline TEXT NOT NULL,
                status TEXT NOT NULL DEFAULT 'active',
                created_at TEXT NOT NULL,
                message_id INTEGER
            )
        """)
        await db.execute("""
            CREATE TABLE IF NOT EXISTS slots (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                meeting_id INTEGER NOT NULL,
                user_id INTEGER NOT NULL,
                slot_datetime TEXT NOT NULL,
                FOREIGN KEY (meeting_id) REFERENCES meetings(id)
            )
        """)
        await db.execute("""
            CREATE TABLE IF NOT EXISTS participants (
                meeting_id INTEGER NOT NULL,
                user_id INTEGER NOT NULL,
                username TEXT,
                PRIMARY KEY (meeting_id, user_id),
                FOREIGN KEY (meeting_id) REFERENCES meetings(id)
            )
        """)
        await db.commit()
    logger.info("База данных инициализирована.")


# ─── MEETINGS ────────────────────────────────────────────────────────────────

async def create_meeting(
    title: str,
    group_chat_id: int,
    created_by: int,
    deadline: str,
    created_at: str,
) -> int:
    async with aiosqlite.connect(DB_PATH) as db:
        cursor = await db.execute(
            """
            INSERT INTO meetings (title, group_chat_id, created_by, deadline, status, created_at)
            VALUES (?, ?, ?, ?, 'active', ?)
            """,
            (title, group_chat_id, created_by, deadline, created_at),
        )
        await db.commit()
        return cursor.lastrowid


async def get_meeting(meeting_id: int) -> Optional[dict]:
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        async with db.execute(
            "SELECT * FROM meetings WHERE id = ?", (meeting_id,)
        ) as cursor:
            row = await cursor.fetchone()
            return dict(row) if row else None


async def get_active_meetings_by_group(group_chat_id: int) -> list[dict]:
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        async with db.execute(
            "SELECT * FROM meetings WHERE group_chat_id = ? AND status = 'active'",
            (group_chat_id,),
        ) as cursor:
            rows = await cursor.fetchall()
            return [dict(r) for r in rows]


async def get_all_active_meetings() -> list[dict]:
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        async with db.execute(
            "SELECT * FROM meetings WHERE status = 'active'"
        ) as cursor:
            rows = await cursor.fetchall()
            return [dict(r) for r in rows]


async def close_meeting(meeting_id: int) -> None:
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            "UPDATE meetings SET status = 'closed' WHERE id = ?", (meeting_id,)
        )
        await db.commit()


async def cancel_meeting(meeting_id: int) -> None:
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            "UPDATE meetings SET status = 'cancelled' WHERE id = ?", (meeting_id,)
        )
        await db.commit()


async def save_message_id(meeting_id: int, message_id: int) -> None:
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            "UPDATE meetings SET message_id = ? WHERE id = ?",
            (message_id, meeting_id),
        )
        await db.commit()


# ─── SLOTS ───────────────────────────────────────────────────────────────────

async def add_user_slot(meeting_id: int, user_id: int, slot_datetime: str) -> int:
    """Добавляет слот конкретного участника."""
    async with aiosqlite.connect(DB_PATH) as db:
        cursor = await db.execute(
            "INSERT INTO slots (meeting_id, user_id, slot_datetime) VALUES (?, ?, ?)",
            (meeting_id, user_id, slot_datetime),
        )
        await db.commit()
        return cursor.lastrowid


async def delete_user_slots(meeting_id: int, user_id: int) -> None:
    """Удаляет все слоты участника перед повторным сохранением."""
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            "DELETE FROM slots WHERE meeting_id = ? AND user_id = ?",
            (meeting_id, user_id),
        )
        await db.commit()


async def get_user_slots(meeting_id: int, user_id: int) -> list[str]:
    """Возвращает список слотов конкретного участника."""
    async with aiosqlite.connect(DB_PATH) as db:
        async with db.execute(
            "SELECT slot_datetime FROM slots WHERE meeting_id = ? AND user_id = ? ORDER BY slot_datetime",
            (meeting_id, user_id),
        ) as cursor:
            rows = await cursor.fetchall()
            return [row[0] for row in rows]


async def get_all_slots_for_meeting(meeting_id: int) -> list[dict]:
    """Возвращает все слоты всех участников встречи."""
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        async with db.execute(
            "SELECT * FROM slots WHERE meeting_id = ? ORDER BY slot_datetime",
            (meeting_id,),
        ) as cursor:
            rows = await cursor.fetchall()
            return [dict(r) for r in rows]


# ─── PARTICIPANTS ─────────────────────────────────────────────────────────────

async def upsert_participant(meeting_id: int, user_id: int, username: str) -> None:
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            """
            INSERT OR REPLACE INTO participants (meeting_id, user_id, username)
            VALUES (?, ?, ?)
            """,
            (meeting_id, user_id, username),
        )
        await db.commit()


async def get_participants(meeting_id: int) -> list[dict]:
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        async with db.execute(
            "SELECT * FROM participants WHERE meeting_id = ?", (meeting_id,)
        ) as cursor:
            rows = await cursor.fetchall()
            return [dict(r) for r in rows]


async def count_participants(meeting_id: int) -> int:
    async with aiosqlite.connect(DB_PATH) as db:
        async with db.execute(
            "SELECT COUNT(*) FROM participants WHERE meeting_id = ?", (meeting_id,)
        ) as cursor:
            row = await cursor.fetchone()
            return row[0] if row else 0


# ─── RESULTS ─────────────────────────────────────────────────────────────────

async def get_common_slots(meeting_id: int) -> list[str]:
    """
    Возвращает слоты, которые отметили ВСЕ участники встречи.
    Если участников 0 — возвращает пустой список.
    """
    async with aiosqlite.connect(DB_PATH) as db:
        # Считаем количество уникальных участников
        async with db.execute(
            "SELECT COUNT(DISTINCT user_id) FROM participants WHERE meeting_id = ?",
            (meeting_id,),
        ) as cursor:
            row = await cursor.fetchone()
            total = row[0] if row else 0

        if total == 0:
            return []

        # Находим слоты, которые встречаются у всех участников
        async with db.execute(
            """
            SELECT slot_datetime
            FROM slots
            WHERE meeting_id = ?
            GROUP BY slot_datetime
            HAVING COUNT(DISTINCT user_id) = ?
            ORDER BY slot_datetime
            """,
            (meeting_id, total),
        ) as cursor:
            rows = await cursor.fetchall()
            return [row[0] for row in rows]
