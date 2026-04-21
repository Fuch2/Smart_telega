import aiosqlite
import logging
from typing import Optional
from config import DB_PATH

logger = logging.getLogger(__name__)


async def _ensure_column(db: aiosqlite.Connection, table: str, column_def: str) -> None:
    column_name = column_def.split()[0]
    async with db.execute(f"PRAGMA table_info({table})") as cursor:
        rows = await cursor.fetchall()
    existing_columns = {row[1] for row in rows}
    if column_name not in existing_columns:
        await db.execute(f"ALTER TABLE {table} ADD COLUMN {column_def}")


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
        await _ensure_column(db, "meetings", "final_slot_datetime TEXT")
        await _ensure_column(db, "meetings", "finalized_at TEXT")
        await _ensure_column(db, "meetings", "finalized_by INTEGER")
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
        await db.execute("""
            CREATE TABLE IF NOT EXISTS user_integrations (
                user_id INTEGER NOT NULL,
                provider TEXT NOT NULL,
                access_token TEXT,
                refresh_token TEXT,
                expires_at TEXT,
                scope TEXT,
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                PRIMARY KEY (user_id, provider)
            )
        """)
        await db.execute("""
            CREATE TABLE IF NOT EXISTS meeting_exports (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                meeting_id INTEGER NOT NULL,
                user_id INTEGER NOT NULL,
                provider TEXT NOT NULL,
                external_id TEXT,
                slot_datetime TEXT,
                status TEXT NOT NULL DEFAULT 'pending',
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                FOREIGN KEY (meeting_id) REFERENCES meetings(id)
            )
        """)
        await db.execute("""
            CREATE TABLE IF NOT EXISTS meeting_custom_slots (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                meeting_id INTEGER NOT NULL,
                slot_datetime TEXT NOT NULL,
                created_by INTEGER NOT NULL,
                UNIQUE (meeting_id, slot_datetime),
                FOREIGN KEY (meeting_id) REFERENCES meetings(id)
            )
        """)
        await db.execute("""
            CREATE TABLE IF NOT EXISTS meeting_access (
                meeting_id INTEGER NOT NULL,
                user_id INTEGER NOT NULL,
                accessed_at TEXT NOT NULL,
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


async def finalize_meeting(
    meeting_id: int,
    finalized_at: str,
    final_slot_datetime: str | None = None,
    finalized_by: int | None = None,
) -> None:
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            """
            UPDATE meetings
            SET status = 'closed',
                final_slot_datetime = ?,
                finalized_at = ?,
                finalized_by = ?
            WHERE id = ?
            """,
            (final_slot_datetime, finalized_at, finalized_by, meeting_id),
        )
        await db.commit()


async def remember_meeting_access(meeting_id: int, user_id: int, accessed_at: str) -> None:
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            """
            INSERT INTO meeting_access (meeting_id, user_id, accessed_at)
            VALUES (?, ?, ?)
            ON CONFLICT(meeting_id, user_id)
            DO UPDATE SET accessed_at = excluded.accessed_at
            """,
            (meeting_id, user_id, accessed_at),
        )
        await db.commit()


async def get_user_active_meetings(user_id: int) -> list[dict]:
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        async with db.execute(
            """
            SELECT DISTINCT
                m.*,
                CASE WHEN m.created_by = ? THEN 1 ELSE 0 END AS is_creator,
                CASE WHEN p.user_id IS NOT NULL THEN 1 ELSE 0 END AS has_voted,
                CASE WHEN a.user_id IS NOT NULL THEN 1 ELSE 0 END AS has_opened
            FROM meetings m
            LEFT JOIN participants p
              ON p.meeting_id = m.id
             AND p.user_id = ?
            LEFT JOIN meeting_access a
              ON a.meeting_id = m.id
             AND a.user_id = ?
            WHERE m.status = 'active'
              AND (
                  m.created_by = ?
                  OR p.user_id IS NOT NULL
                  OR a.user_id IS NOT NULL
              )
            ORDER BY m.deadline ASC, m.created_at DESC
            """,
            (user_id, user_id, user_id, user_id),
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


async def add_meeting_custom_slot(meeting_id: int, slot_datetime: str, created_by: int) -> None:
    """Добавляет пользовательский слот для встречи, если его ещё нет."""
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            """
            INSERT OR IGNORE INTO meeting_custom_slots (meeting_id, slot_datetime, created_by)
            VALUES (?, ?, ?)
            """,
            (meeting_id, slot_datetime, created_by),
        )
        await db.commit()


async def get_meeting_custom_slots(meeting_id: int) -> list[str]:
    """Возвращает все дополнительные слоты встречи."""
    async with aiosqlite.connect(DB_PATH) as db:
        async with db.execute(
            """
            SELECT slot_datetime
            FROM meeting_custom_slots
            WHERE meeting_id = ?
            ORDER BY slot_datetime
            """,
            (meeting_id,),
        ) as cursor:
            rows = await cursor.fetchall()
            return [row[0] for row in rows]


# ─── PARTICIPANTS ─────────────────────────────────────────────────────────────

async def get_user_integrations(user_id: int) -> list[dict]:
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        async with db.execute(
            """
            SELECT provider, access_token, refresh_token, expires_at, scope, created_at, updated_at
            FROM user_integrations
            WHERE user_id = ?
            ORDER BY provider
            """,
            (user_id,),
        ) as cursor:
            rows = await cursor.fetchall()
            return [dict(r) for r in rows]


async def save_user_integration(
    user_id: int,
    provider: str,
    created_at: str,
    access_token: str | None = None,
    refresh_token: str | None = None,
    expires_at: str | None = None,
    scope: str | None = None,
) -> None:
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            """
            INSERT INTO user_integrations (
                user_id, provider, access_token, refresh_token, expires_at, scope, created_at, updated_at
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT(user_id, provider)
            DO UPDATE SET
                access_token = excluded.access_token,
                refresh_token = excluded.refresh_token,
                expires_at = excluded.expires_at,
                scope = excluded.scope,
                updated_at = excluded.updated_at
            """,
            (
                user_id,
                provider,
                access_token,
                refresh_token,
                expires_at,
                scope,
                created_at,
                created_at,
            ),
        )
        await db.commit()


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


async def get_participant_votes(meeting_id: int) -> list[dict]:
    """Возвращает участников встречи вместе с выбранными слотами."""
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        async with db.execute(
            """
            SELECT
                p.meeting_id,
                p.user_id,
                p.username,
                s.slot_datetime
            FROM participants p
            LEFT JOIN slots s
              ON s.meeting_id = p.meeting_id
             AND s.user_id = p.user_id
            WHERE p.meeting_id = ?
            ORDER BY LOWER(COALESCE(p.username, '')), p.user_id, s.slot_datetime
            """,
            (meeting_id,),
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
