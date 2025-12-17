"""
Chat module for RAG Chatbot
Handles RAG logic using OpenAI, selected-text-only mode, and citations
"""
import os
import uuid
import traceback
from typing import Optional, List, Dict
from dotenv import load_dotenv
import google.generativeai as genai
from embedding import search_embeddings
from db import db_manager
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

# Initialize Google Gemini client
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
gemini_model = genai.GenerativeModel('gemini-flash-latest')  # Using the latest flash model available

async def generate_response(question: str, selected_text: Optional[str] = None, session_id: Optional[str] = None):
    """
    Generate response using RAG logic

    Args:
        question: User's question
        selected_text: Optional text to search within only (selected-text-only mode)
        session_id: Session identifier

    Returns:
        dict: Response with answer and citations
    """
    try:
        session_id = session_id or str(uuid.uuid4())

        # CCR: Wrap initial session save in try-except with full traceback logging
        try:
            await db_manager.save_session(session_id)
        except Exception as db_error:
            logger.error(f"Initial session save error: {str(db_error)}")
            logger.error(f"Full traceback:\n{traceback.format_exc()}")
            # Continue processing even if session saving fails

        # Validate and handle selected text
        if selected_text is not None:
            selected_text = selected_text.strip()

            logger.info(f"Raw selected text: '{selected_text[:100]}...' (length: {len(selected_text)})")

            # Clean the selected text by removing UI elements and artifacts
            # Remove common UI artifacts that might be selected along with text
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

        # Determine search text based on mode
        if selected_text:
            # Selected-text-only mode - only search within the provided text
            search_text = selected_text
        else:
            # Regular mode - use the question for semantic search
            search_text = question

        # CCR: Wrap vector search in try-except with full traceback logging
        try:
            search_result = search_embeddings(search_text, limit=5, score_threshold=0.3)
            relevant_texts = search_result["relevant_texts"]
            citations = search_result["citations"]
        except Exception as vector_error:
            logger.error(f"Vector search error: {str(vector_error)}")
            logger.error(f"Full traceback:\n{traceback.format_exc()}")
            # Return safe fallback response
            fallback_session_id = session_id or str(uuid.uuid4())
            error_msg = f"Debug info: Vector search failed - {str(vector_error)}"
            # Attempt to save to database
            try:
                await db_manager.save_question_answer(fallback_session_id, question, error_msg)
            except Exception as db_error:
                logger.error(f"Database save error during fallback: {str(db_error)}")
                logger.error(f"Full traceback:\n{traceback.format_exc()}")
            return {
                "answer": error_msg,
                "citations": [],
                "session_id": fallback_session_id
            }

        if not relevant_texts:
            answer = "I couldn't find relevant information in the book to answer your question."
            if selected_text and len(selected_text) >= 10:
                answer = f"I couldn't find relevant information in the provided text to answer your question. The provided text may be too short or not contain the information needed: '{selected_text[:100]}...'"
            # CCR: Wrap database save in try-except with full traceback logging
            try:
                await db_manager.save_question_answer(session_id, question, answer)
            except Exception as db_error:
                logger.error(f"Database save error when no relevant texts found: {str(db_error)}")
                logger.error(f"Full traceback:\n{traceback.format_exc()}")
                # Continue with returning the answer even if DB save fails
            return {
                "answer": answer,
                "citations": [],
                "session_id": session_id
            }

        # Combine retrieved texts for context
        context = "\n\n".join(relevant_texts)

        # Prepare the prompt for OpenAI
        if selected_text:
            # Selected-text-only mode
            prompt = f"""Based solely on the following text from the book, answer the question:

Text: {selected_text}

Question: {question}

Answer the question based only on the provided text. If the text doesn't contain enough information to answer the question, say so."""
        else:
            # Regular RAG mode
            prompt = f"""Based on the following context from the book, answer the question:

Context: {context}

Question: {question}

Provide a comprehensive answer based on the context. Include relevant citations from the context. If the context doesn't contain enough information to answer the question, say so."""

        # Call Google Gemini API to generate the answer
        try:
            # Configure generation parameters
            generation_config = {
                "temperature": 0.3,
                "max_output_tokens": 1000,
            }

            # Create the system context and user prompt
            system_context = "You are a helpful assistant that answers questions based on the provided context from a book about physical AI and humanoid robotics. Be precise, cite relevant parts of the context, and acknowledge when the context doesn't contain enough information to answer."

            # Combine system context with the prompt
            full_prompt = f"{system_context}\n\n{prompt}"

            # Generate content using Gemini
            response = gemini_model.generate_content(
                full_prompt,
                generation_config=generation_config
            )

            answer = response.text if response.text else "I couldn't generate a response based on the provided context."
        except Exception as api_error:
            logger.error(f"Gemini API error: {str(api_error)}")
            logger.error(f"Full traceback:\n{traceback.format_exc()}")
            answer = f"Debug info: Gemini API failed - {str(api_error)}"
            # CCR: Wrap database save in try-except with full traceback logging
            try:
                await db_manager.save_question_answer(session_id, question, answer)
            except Exception as db_error:
                logger.error(f"Database save error during Gemini API fallback: {str(db_error)}")
                logger.error(f"Full traceback:\n{traceback.format_exc()}")
                # Continue with returning the answer even if DB save fails
            return {
                "answer": answer,
                "citations": citations,
                "session_id": session_id
            }

        # CCR: Wrap final database save in try-except with full traceback logging
        try:
            await db_manager.save_question_answer(session_id, question, answer)
        except Exception as db_error:
            logger.error(f"Final database save error: {str(db_error)}")
            logger.error(f"Full traceback:\n{traceback.format_exc()}")
            # Continue returning the answer even if saving fails

        return {
            "answer": answer,
            "citations": citations,
            "session_id": session_id
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
            # If even saving to DB fails, just continue with returning the answer

        # Return a safe response instead of raising an exception
        return {
            "answer": error_answer,
            "citations": [],
            "session_id": fallback_session_id
        }