"""
Database module for RAG Chatbot
Handles Neon Postgres connection and session/question tracking
"""
import os
import asyncpg
import psycopg2
from typing import Optional
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DatabaseManager:
    def __init__(self):
        self.database_url = os.getenv("NEON_DATABASE_URL")
        if not self.database_url:
            logger.warning("NEON_DATABASE_URL not set. Database functionality will be disabled.")

    async def get_connection(self):
        """Get an async connection to the PostgreSQL database"""
        if not self.database_url:
            return None

        try:
            conn = await asyncpg.connect(dsn=self.database_url)
            return conn
        except Exception as e:
            logger.error(f"Failed to connect to database: {e}")
            return None

    def get_sync_connection(self):
        """Get a synchronous connection to the PostgreSQL database"""
        if not self.database_url:
            return None

        try:
            conn = psycopg2.connect(self.database_url)
            return conn
        except Exception as e:
            logger.error(f"Failed to connect to database synchronously: {e}")
            return None

    async def initialize_tables(self):
        """Create required tables if they don't exist"""
        if not self.database_url:
            logger.info("Skipping database initialization - no database URL provided")
            return

        conn = await self.get_connection()
        if not conn:
            return

        try:
            # Create sessions table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS sessions (
                    session_id VARCHAR(255) PRIMARY KEY,
                    user_id VARCHAR(255),
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)

            # Create questions table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS questions (
                    question_id SERIAL PRIMARY KEY,
                    session_id VARCHAR(255) REFERENCES sessions(session_id),
                    question TEXT NOT NULL,
                    answer TEXT NOT NULL,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)

            logger.info("Database tables initialized successfully")
        except Exception as e:
            logger.error(f"Error initializing database tables: {e}")
        finally:
            await conn.close()

    def initialize_tables_sync(self):
        """Create required tables if they don't exist (synchronous version)"""
        if not self.database_url:
            logger.info("Skipping database initialization - no database URL provided")
            return

        conn = self.get_sync_connection()
        if not conn:
            return

        try:
            cursor = conn.cursor()

            # Create sessions table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS sessions (
                    session_id VARCHAR(255) PRIMARY KEY,
                    user_id VARCHAR(255),
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)

            # Create questions table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS questions (
                    question_id SERIAL PRIMARY KEY,
                    session_id VARCHAR(255) REFERENCES sessions(session_id),
                    question TEXT NOT NULL,
                    answer TEXT NOT NULL,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)

            conn.commit()
            logger.info("Database tables initialized successfully (sync)")
        except Exception as e:
            logger.error(f"Error initializing database tables (sync): {e}")
        finally:
            conn.close()

    async def save_session(self, session_id: str, user_id: Optional[str] = None):
        """Save a new session to the database"""
        if not self.database_url:
            return

        conn = await self.get_connection()
        if not conn:
            return

        try:
            await conn.execute("""
                INSERT INTO sessions (session_id, user_id)
                VALUES ($1, $2)
                ON CONFLICT (session_id) DO NOTHING
            """, session_id, user_id or "")
        except Exception as e:
            logger.error(f"Error saving session: {e}")
        finally:
            await conn.close()

    async def save_question_answer(self, session_id: str, question: str, answer: str):
        """Save a question-answer pair to the database"""
        if not self.database_url:
            return

        conn = await self.get_connection()
        if not conn:
            return

        try:
            await conn.execute("""
                INSERT INTO questions (session_id, question, answer)
                VALUES ($1, $2, $3)
            """, session_id, question, answer)
        except Exception as e:
            logger.error(f"Error saving question-answer: {e}")
        finally:
            await conn.close()

    def save_session_sync(self, session_id: str, user_id: Optional[str] = None):
        """Save a new session to the database (synchronous version)"""
        if not self.database_url:
            return

        conn = self.get_sync_connection()
        if not conn:
            return

        try:
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO sessions (session_id, user_id)
                VALUES (%s, %s)
                ON CONFLICT (session_id) DO NOTHING
            """, (session_id, user_id or ""))
            conn.commit()
        except Exception as e:
            logger.error(f"Error saving session (sync): {e}")
        finally:
            conn.close()

    def save_question_answer_sync(self, session_id: str, question: str, answer: str):
        """Save a question-answer pair to the database (synchronous version)"""
        if not self.database_url:
            return

        conn = self.get_sync_connection()
        if not conn:
            return

        try:
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO questions (session_id, question, answer)
                VALUES (%s, %s, %s)
            """, (session_id, question, answer))
            conn.commit()
        except Exception as e:
            logger.error(f"Error saving question-answer (sync): {e}")
        finally:
            conn.close()

# Global database manager instance
db_manager = DatabaseManager()