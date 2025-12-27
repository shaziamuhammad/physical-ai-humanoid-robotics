"""
Chat module for RAG Chatbot
Handles proxying requests to external Hugging Face RAG backend
"""
import os
import uuid
import traceback
import httpx
from typing import Optional, List, Dict
from dotenv import load_dotenv
from db import db_manager
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

# Hugging Face RAG backend configuration
HUGGINGFACE_RAG_URL = os.getenv("HUGGINGFACE_RAG_URL", "https://shaziabashir-rag-chatbot.hf.space")

async def generate_response(question: str, selected_text: Optional[str] = None, session_id: Optional[str] = None):
    """
    Generate response by proxying to external Hugging Face RAG backend

    Args:
        question: User's question
        selected_text: Optional text to search within only (selected-text-only mode)
        session_id: Session identifier

    Returns:
        dict: Response with answer and citations
    """
    try:
        # Validate and handle selected text (similar to original logic)
        if selected_text is not None:
            selected_text = selected_text.strip()

            logger.info(f"Raw selected text: '{selected_text[:100]}...' (length: {len(selected_text)})")

            # Clean the selected text by removing UI elements and artifacts
            # Remove common UI artifacts that might be accidentally selected
            import re
            # Remove emoji patterns and common UI elements that might be accidentally selected
            cleaned_text = re.sub(r'[📝🗑️×\n\r\t]+', ' ', selected_text)
            cleaned_text = re.sub(r'\s+', ' ', cleaned_text).strip()  # Normalize whitespace

            logger.info(f"Cleaned selected text: '{cleaned_text[:100]}...' (length: {len(cleaned_text)})")

            # Additional cleaning: remove any text that looks like chatbot UI elements
            if 'Selected text:' in cleaned_text or 'Mode:' in cleaned_text:
                logger.warning("Detected recursive chatbot UI text in selection, ignoring")
                selected_text = None
            elif not cleaned_text:
                # If cleaned text is empty, fall back to regular mode with a helpful message
                logger.warning("Selected text is empty after cleaning, falling back to regular RAG mode")
                selected_text = None
            elif len(cleaned_text) < 10:  # Minimum length check
                # If cleaned text is too short, fall back to regular mode with a helpful message
                logger.warning(f"Selected text is too short after cleaning (length: {len(cleaned_text)}), falling back to regular RAG mode")
                selected_text = None
            else:
                selected_text = cleaned_text
                logger.info(f"Using selected text mode with {len(selected_text)} characters")

        # Prepare the request to the Hugging Face backend
        payload = {
            "question": question,
            "selected_text": selected_text,
            "session_id": session_id
        }

        # Make the request to the Hugging Face backend
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(
                f"{HUGGINGFACE_RAG_URL}/chat",
                json=payload,
                headers={"Content-Type": "application/json"}
            )

            if response.status_code != 200:
                raise Exception(f"HTTP error from Hugging Face backend: {response.status_code}")

            hf_response = response.json()

        # Extract the response data from Hugging Face backend
        answer = hf_response.get("answer", "I couldn't generate a response.")
        citations = hf_response.get("citations", [])
        returned_session_id = hf_response.get("session_id", session_id or str(uuid.uuid4()))

        # Use the session ID from the backend response or generate a new one
        final_session_id = returned_session_id

        # Save the session and question-answer pair to the database
        try:
            await db_manager.save_session(final_session_id)
            await db_manager.save_question_answer(final_session_id, question, answer)
        except Exception as db_error:
            logger.error(f"Database save error: {str(db_error)}")
            logger.error(f"Full traceback:\n{traceback.format_exc()}")
            # Continue with returning the answer even if DB save fails

        return {
            "answer": answer,
            "citations": citations,
            "session_id": final_session_id
        }

    except httpx.RequestError as e:
        logger.error(f"Request error to Hugging Face backend: {str(e)}")
        logger.error(f"Full traceback:\n{traceback.format_exc()}")
        fallback_session_id = session_id or str(uuid.uuid4())
        error_answer = f"Error connecting to the RAG backend: {str(e)}"

        # Try to save to database
        try:
            await db_manager.save_question_answer(fallback_session_id, question, error_answer)
        except Exception as db_error:
            logger.error(f"Database save error during fallback: {str(db_error)}")

        return {
            "answer": error_answer,
            "citations": [],
            "session_id": fallback_session_id
        }
    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        logger.error(f"Full traceback:\n{traceback.format_exc()}")
        # Generate a safe fallback session ID in case it wasn't created properly
        fallback_session_id = session_id or str(uuid.uuid4())
        # Save error to database with debug information
        error_answer = f"Debug info: {str(e)}"
        try:
            await db_manager.save_question_answer(fallback_session_id, question, error_answer)
        except Exception as db_error:
            logger.error(f"Database save error in top-level exception: {str(db_error)}")
            logger.error(f"Full traceback:\n{traceback.format_exc()}")

        # Return a safe response instead of raising an exception
        return {
            "answer": error_answer,
            "citations": [],
            "session_id": fallback_session_id
        }