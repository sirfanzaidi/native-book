"""
Neon Serverless Postgres database connection and operations.

This module provides async database connection pooling and
session management for the RAG chatbot.
"""

import os
from datetime import datetime, timedelta
from typing import Optional
from uuid import UUID, uuid4

import asyncpg
from pydantic import BaseModel
from pydantic_settings import BaseSettings

from src.config.logging import get_logger

logger = get_logger(__name__)


class DatabaseConfig(BaseSettings):
    """Configuration for Neon Postgres database."""

    neon_database_url: str
    neon_pool_min_size: int = 1
    neon_pool_max_size: int = 10

    class Config:
        """Pydantic config."""

        env_file = ".env"
        case_sensitive = False
        extra = "ignore"  # Ignore extra environment variables


class ChatSession(BaseModel):
    """Chat session model."""

    session_id: UUID
    mode: str  # 'full_book' or 'selected_text'
    selected_text: Optional[str] = None
    created_at: datetime
    expires_at: datetime


class DatabaseService:
    """
    Neon Postgres database service.

    Provides connection pooling and CRUD operations for
    chat sessions and metadata.
    """

    def __init__(self, config: Optional[DatabaseConfig] = None) -> None:
        """
        Initialize database service with configuration.

        Args:
            config: Database configuration (uses environment if None)
        """
        self.config = config or DatabaseConfig(
            neon_database_url=os.getenv("NEON_DATABASE_URL", ""),
        )

        self.pool: Optional[asyncpg.Pool] = None

        logger.info(
            "database_service_initialized",
            pool_min_size=self.config.neon_pool_min_size,
            pool_max_size=self.config.neon_pool_max_size,
        )

    async def connect(self) -> None:
        """
        Create database connection pool.

        Creates the chat_sessions table if it doesn't exist.
        """
        self.pool = await asyncpg.create_pool(
            self.config.neon_database_url,
            min_size=self.config.neon_pool_min_size,
            max_size=self.config.neon_pool_max_size,
        )

        # Create chat_sessions table if not exists
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                CREATE TABLE IF NOT EXISTS chat_sessions (
                    session_id UUID PRIMARY KEY,
                    mode VARCHAR(20) NOT NULL CHECK (mode IN ('full_book', 'selected_text')),
                    selected_text TEXT,
                    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
                    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
                    CONSTRAINT expires_after_created CHECK (expires_at > created_at)
                );
                """
            )

            # Create index on expires_at for cleanup queries
            await conn.execute(
                """
                CREATE INDEX IF NOT EXISTS idx_chat_sessions_expires_at
                ON chat_sessions(expires_at);
                """
            )

        logger.info("database_connected", message="Connection pool created and schema initialized")

    async def close(self) -> None:
        """Close database connection pool."""
        if self.pool:
            await self.pool.close()
            logger.info("database_closed")

    async def create_session(
        self,
        mode: str,
        selected_text: Optional[str] = None,
        expiry_hours: int = 24,
    ) -> ChatSession:
        """
        Create a new chat session.

        Args:
            mode: Session mode ('full_book' or 'selected_text')
            selected_text: Text selected by user (required for selected_text mode)
            expiry_hours: Session expiry in hours (default: 24)

        Returns:
            Created ChatSession object

        Raises:
            ValueError: If pool not initialized or invalid mode
        """
        if not self.pool:
            raise ValueError("Database pool not initialized. Call connect() first.")

        if mode not in ("full_book", "selected_text"):
            raise ValueError(f"Invalid mode: {mode}. Must be 'full_book' or 'selected_text'")

        session_id = uuid4()
        created_at = datetime.utcnow()
        expires_at = created_at + timedelta(hours=expiry_hours)

        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO chat_sessions (session_id, mode, selected_text, created_at, expires_at)
                VALUES ($1, $2, $3, $4, $5)
                """,
                session_id,
                mode,
                selected_text,
                created_at,
                expires_at,
            )

        logger.info(
            "session_created",
            session_id=str(session_id),
            mode=mode,
            expires_at=expires_at.isoformat(),
        )

        return ChatSession(
            session_id=session_id,
            mode=mode,
            selected_text=selected_text,
            created_at=created_at,
            expires_at=expires_at,
        )

    async def get_session(self, session_id: UUID) -> Optional[ChatSession]:
        """
        Retrieve a chat session by ID.

        Args:
            session_id: Session UUID

        Returns:
            ChatSession if found and not expired, None otherwise
        """
        if not self.pool:
            raise ValueError("Database pool not initialized. Call connect() first.")

        async with self.pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                SELECT session_id, mode, selected_text, created_at, expires_at
                FROM chat_sessions
                WHERE session_id = $1 AND expires_at > NOW()
                """,
                session_id,
            )

        if row:
            logger.info(
                "session_retrieved",
                session_id=str(session_id),
                mode=row["mode"],
            )

            return ChatSession(
                session_id=row["session_id"],
                mode=row["mode"],
                selected_text=row["selected_text"],
                created_at=row["created_at"],
                expires_at=row["expires_at"],
            )

        logger.warning(
            "session_not_found_or_expired",
            session_id=str(session_id),
        )
        return None

    async def delete_expired_sessions(self) -> int:
        """
        Delete all expired sessions.

        Returns:
            Number of sessions deleted
        """
        if not self.pool:
            raise ValueError("Database pool not initialized. Call connect() first.")

        async with self.pool.acquire() as conn:
            result = await conn.execute(
                """
                DELETE FROM chat_sessions
                WHERE expires_at <= NOW()
                """
            )

        # Extract count from result string "DELETE N"
        count = int(result.split()[-1]) if result else 0

        logger.info(
            "expired_sessions_deleted",
            count=count,
        )

        return count

    async def health_check(self) -> bool:
        """
        Check if database is accessible.

        Returns:
            True if database is accessible, False otherwise
        """
        if not self.pool:
            logger.error("database_health_check", status="unhealthy", reason="pool_not_initialized")
            return False

        try:
            async with self.pool.acquire() as conn:
                await conn.fetchval("SELECT 1")

            logger.info("database_health_check", status="healthy")
            return True

        except Exception as e:
            logger.error(
                "database_health_check",
                status="unhealthy",
                error=str(e),
            )
            return False
