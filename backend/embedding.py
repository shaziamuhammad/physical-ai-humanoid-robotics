"""
Embedding module for RAG Chatbot
Handles text embedding using Cohere and storage in Qdrant
"""
import requests
from bs4 import BeautifulSoup
import xml.etree.ElementTree as ET
from typing import List, Optional
import cohere
import qdrant_client
from qdrant_client.http import models
import uuid
import tiktoken
import os
import logging
import traceback
from dotenv import load_dotenv

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

# Initialize Cohere client
co = cohere.Client(os.getenv("COHERE_API_KEY"))

# Initialize Qdrant client
qdrant = qdrant_client.QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Collection name for storing embeddings
COLLECTION_NAME = "book_embeddings"

def fetch_sitemap_urls(sitemap_url: str) -> List[str]:
    """
    CCR: Fetch all chapter URLs from the book sitemap
    """
    try:
        logger.info(f"Fetching sitemap from: {sitemap_url}")
        response = requests.get(sitemap_url)
        response.raise_for_status()

        # Parse the XML sitemap
        root = ET.fromstring(response.content)

        # Extract URLs from the sitemap
        urls = []
        for url in root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}url/{http://www.sitemaps.org/schemas/sitemap/0.9}loc'):
            urls.append(url.text)

        logger.info(f"Found {len(urls)} URLs in sitemap")
        return urls
    except Exception as e:
        logger.error(f"Error fetching sitemap: {str(e)}")
        logger.error(f"Full traceback:\n{traceback.format_exc()}")
        raise Exception(f"Error fetching sitemap: {str(e)}")

def extract_clean_text_from_html(html_content: str) -> str:
    """
    CCR: Extract clean text from HTML content, removing scripts, styles, navigation, etc.
    """
    try:
        soup = BeautifulSoup(html_content, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
            script.decompose()

        # Get text content
        text = soup.get_text()

        # Clean up whitespace
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = ' '.join(chunk for chunk in chunks if chunk)

        logger.info(f"Extracted clean text of length: {len(text)} characters")
        return text
    except Exception as e:
        logger.error(f"Error extracting text from HTML: {str(e)}")
        logger.error(f"Full traceback:\n{traceback.format_exc()}")
        raise Exception(f"Error extracting text from HTML: {str(e)}")

def fetch_chapter_content(url: str) -> str:
    """
    CCR: Fetch HTML content of a single chapter and extract clean text
    """
    try:
        # Fix the domain - replace github.io domain with vercel.app domain
        corrected_url = url.replace("shaziamuhammad.github.io", "physical-ai-humanoid-robotics-lac.vercel.app")
        logger.info(f"Fetching chapter content from: {corrected_url}")
        response = requests.get(corrected_url)
        response.raise_for_status()

        # Extract clean text from the HTML
        clean_text = extract_clean_text_from_html(response.text)

        logger.info(f"Successfully fetched and processed chapter: {corrected_url}")
        return clean_text
    except Exception as e:
        logger.error(f"Error fetching chapter content from {corrected_url}: {str(e)}")
        logger.error(f"Full traceback:\n{traceback.format_exc()}")
        raise Exception(f"Error fetching chapter content: {str(e)}")

def populate_book_embeddings():
    """
    CCR: Main function to fetch all book content and populate embeddings in Qdrant
    """
    try:
        logger.info("Starting to populate book embeddings from sitemap...")

        # Fetch all URLs from the sitemap
        sitemap_url = "https://physical-ai-humanoid-robotics-lac.vercel.app/sitemap.xml"
        all_urls = fetch_sitemap_urls(sitemap_url)

        # Filter only the book content URLs (those under /docs/)
        book_urls = [url for url in all_urls if '/docs/' in url and not url.startswith('https://shaziamuhammad.github.io/docs/tutorial')]
        logger.info(f"Found {len(book_urls)} book content URLs out of {len(all_urls)} total URLs")

        # Process each book chapter
        processed_count = 0
        for i, url in enumerate(book_urls):
            try:
                logger.info(f"Processing chapter {i+1}/{len(book_urls)}: {url}")

                # Fetch and clean the chapter content
                chapter_content = fetch_chapter_content(url)

                if not chapter_content.strip():
                    logger.warning(f"Chapter {url} has no content, skipping...")
                    continue

                # Generate a unique chapter ID
                chapter_id = f"book_{url.split('/')[-1] or 'index'}"

                # Embed and store the chapter content
                result = embed_and_store(chapter_content, chapter_id, offset=0)
                logger.info(f"Successfully processed chapter {chapter_id}: {result}")
                processed_count += 1

            except Exception as chapter_error:
                logger.error(f"Error processing chapter {url}: {str(chapter_error)}")
                logger.error(f"Full traceback:\n{traceback.format_exc()}")
                # Continue with other chapters even if one fails
                continue

        logger.info(f"Finished populating book embeddings. Processed {processed_count} chapters.")
        return {"status": "success", "chapters_processed": processed_count}

    except Exception as e:
        logger.error(f"Error populating book embeddings: {str(e)}")
        logger.error(f"Full traceback:\n{traceback.format_exc()}")
        raise Exception(f"Error populating book embeddings: {str(e)}")

def initialize_collection():
    """Create collection if it doesn't exist"""
    try:
        qdrant.get_collection(COLLECTION_NAME)
    except Exception as e:
        logger.info(f"Collection {COLLECTION_NAME} does not exist, creating it...")
        try:
            qdrant.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),  # Cohere embeddings are 1024-dimensional
            )
            logger.info(f"Successfully created collection {COLLECTION_NAME}")
        except Exception as create_error:
            logger.error(f"Error creating Qdrant collection: {str(create_error)}")
            logger.error(f"Full traceback:\n{traceback.format_exc()}")
            raise Exception(f"Error creating Qdrant collection: {str(create_error)}")

def chunk_text(text: str, max_tokens: int = 500) -> List[str]:
    """Split text into chunks of approximately max_tokens"""
    tokenizer = tiktoken.encoding_for_model("gpt-3.5-turbo")
    tokens = tokenizer.encode(text)

    chunks = []
    for i in range(0, len(tokens), max_tokens):
        chunk_tokens = tokens[i:i + max_tokens]
        chunk_text = tokenizer.decode(chunk_tokens)
        chunks.append(chunk_text)

    return chunks

def embed_and_store(text: str, chapter_id: str, offset: int = 0) -> dict:
    """
    Embed text and store in Qdrant vector database

    Args:
        text: Text to embed and store
        chapter_id: Chapter identifier
        offset: Starting offset for chunking

    Returns:
        dict: Result with status and number of chunks processed
    """
    try:
        # Validate inputs
        if not text or not text.strip():
            raise ValueError("Text cannot be empty or None")

        if not chapter_id or not chapter_id.strip():
            raise ValueError("Chapter ID cannot be empty or None")

        # Split text into chunks if it's too long
        text_chunks = chunk_text(text, max_tokens=500)

        for i, chunk in enumerate(text_chunks):
            # Skip empty chunks
            if not chunk.strip():
                continue

            # CCR: Wrap Cohere embedding generation in try-except with full traceback logging
            try:
                response = co.embed(
                    texts=[chunk],
                    model="embed-multilingual-v3.0",  # Using multilingual model for broader language support
                    input_type="search_document"
                )
                embedding = response.embeddings[0]
            except Exception as embed_error:
                logger.error(f"Cohere embedding error for chunk {i} of chapter {chapter_id}: {str(embed_error)}")
                logger.error(f"Full traceback:\n{traceback.format_exc()}")
                raise Exception(f"Cohere embedding error: {str(embed_error)}")

            # CCR: Wrap Qdrant storage in try-except with full traceback logging
            try:
                point_id = str(uuid.uuid4())
                qdrant.upsert(
                    collection_name=COLLECTION_NAME,
                    points=[
                        models.PointStruct(
                            id=point_id,
                            vector=embedding,
                            payload={
                                "text": chunk,
                                "chapter_id": chapter_id,
                                "offset": offset + i,
                                "original_text_length": len(chunk)
                            }
                        )
                    ]
                )
            except Exception as qdrant_error:
                logger.error(f"Qdrant storage error for chunk {i} of chapter {chapter_id}: {str(qdrant_error)}")
                logger.error(f"Full traceback:\n{traceback.format_exc()}")
                raise Exception(f"Qdrant storage error: {str(qdrant_error)}")

        return {"status": "success", "chapter_id": chapter_id, "chunks_processed": len(text_chunks)}

    except ValueError as ve:
        logger.error(f"Validation error in embed_and_store: {str(ve)}")
        logger.error(f"Full traceback:\n{traceback.format_exc()}")
        raise Exception(f"Validation error: {str(ve)}")
    except Exception as e:
        logger.error(f"Error processing embeddings: {str(e)}")
        logger.error(f"Full traceback:\n{traceback.format_exc()}")
        raise Exception(f"Error processing embeddings: {str(e)}")

def search_embeddings(query: str, limit: int = 5, score_threshold: float = 0.3) -> List[dict]:
    """
    Search for relevant embeddings in Qdrant

    Args:
        query: Query text to search for
        limit: Maximum number of results to return
        score_threshold: Minimum similarity score threshold

    Returns:
        List of relevant text chunks with metadata
    """
    try:
        logger.info(f"Searching embeddings for query: '{query[:100]}...' (length: {len(query)})")

        # Clean the query text by removing UI elements and artifacts
        import re
        # Remove emoji patterns and common UI elements that might be accidentally included
        cleaned_query = re.sub(r'[📝🗑️×\n\r\t]+', ' ', query)
        cleaned_query = re.sub(r'\s+', ' ', cleaned_query).strip()  # Normalize whitespace

        logger.info(f"Cleaned query: '{cleaned_query[:100]}...' (length: {len(cleaned_query)})")

        if not cleaned_query:
            logger.warning("Query is empty after cleaning")
            return {"relevant_texts": [], "citations": []}

        # CCR: Wrap Cohere embedding generation in try-except with full traceback logging
        try:
            response = co.embed(
                texts=[cleaned_query],
                model="embed-multilingual-v3.0",
                input_type="search_query"
            )
            query_embedding = response.embeddings[0]
        except Exception as embed_error:
            logger.error(f"Cohere embedding error during search: {str(embed_error)}")
            logger.error(f"Full traceback:\n{traceback.format_exc()}")
            raise Exception(f"Cohere embedding error: {str(embed_error)}")

        # CCR: Wrap Qdrant search in try-except with full traceback logging
        try:
            search_results = qdrant.query_points(
                collection_name=COLLECTION_NAME,
                query=query_embedding,
                limit=limit,
                score_threshold=score_threshold
            )
        except Exception as qdrant_error:
            logger.error(f"Qdrant search error: {str(qdrant_error)}")
            logger.error(f"Full traceback:\n{traceback.format_exc()}")
            raise Exception(f"Qdrant search error: {str(qdrant_error)}")

        # Process results
        relevant_texts = []
        citations = []

        # Handle the newer Qdrant query_points API result format
        # The query_points method returns a QueryResponse object - check for different possible attribute names
        if hasattr(search_results, 'results'):
            search_results_list = search_results.results
        elif hasattr(search_results, 'points'):
            search_results_list = search_results.points
        else:
            # Fallback: assume search_results is directly iterable (older format)
            search_results_list = search_results

        for result in search_results_list:
            # Check if result has score attribute (newer API) or if we need to handle differently
            result_score = getattr(result, 'score', 0.0)
            if result_score > score_threshold:
                # Get payload from result (check multiple possible attribute names)
                payload = getattr(result, 'payload', {})
                if isinstance(payload, dict) and 'text' in payload:
                    relevant_texts.append(payload["text"])

                    citations.append({
                        "text": payload["text"][:200] + "...",  # Preview of the cited text
                        "chapter_id": payload.get("chapter_id", "unknown"),
                        "score": result_score
                    })

        logger.info(f"Found {len(relevant_texts)} relevant texts")
        return {"relevant_texts": relevant_texts, "citations": citations}

    except Exception as e:
        logger.error(f"Error searching embeddings: {str(e)}")
        logger.error(f"Full traceback:\n{traceback.format_exc()}")
        raise Exception(f"Error searching embeddings: {str(e)}")