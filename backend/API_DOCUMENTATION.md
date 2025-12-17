# RAG Chatbot API Documentation

## Overview
The RAG Chatbot API provides endpoints for embedding book content and querying it using natural language questions. The API uses Cohere for embeddings, Qdrant for vector storage, and Google Gemini for generating responses.

## Base URL
`http://localhost:8000` (when running locally)

## Endpoints

### GET /
**Health Check**
- **Method**: `GET`
- **URL**: `/`
- **Description**: Check if the API is running
- **Response**:
```json
{
  "status": "healthy",
  "service": "RAG Chatbot API"
}
```

### POST /embed
**Embed Text**
- **Method**: `POST`
- **URL**: `/embed`
- **Description**: Embed text content and store in vector database
- **Headers**:
  - `Content-Type: application/json`
- **Request Body**:
```json
{
  "text": "Text content to embed",
  "chapter_id": "Unique identifier for the chapter",
  "offset": 0
}
```
- **Response**:
```json
{
  "status": "success",
  "chapter_id": "Unique identifier for the chapter",
  "chunks_processed": 3
}
```

### POST /chat
**Chat Query**
- **Method**: `POST`
- **URL**: `/chat`
- **Description**: Query the RAG system with a question
- **Headers**:
  - `Content-Type: application/json`
- **Request Body**:
```json
{
  "question": "Your question about the book",
  "selected_text": "Optional text to search within only",
  "session_id": "Optional session identifier"
}
```
- **Response**:
```json
{
  "answer": "Generated answer based on book content",
  "citations": [
    {
      "text": "Preview of cited text...",
      "chapter_id": "Chapter identifier",
      "score": 0.78
    }
  ],
  "session_id": "Session identifier"
}
```

## Testing Examples

### Using curl

#### Health Check
```bash
curl -X GET http://localhost:8000/
```

#### Embed Text
```bash
curl -X POST http://localhost:8000/embed \
  -H "Content-Type: application/json" \
  -d '{
    "text": "Artificial intelligence is a wonderful field that combines computer science and cognitive abilities...",
    "chapter_id": "chapter-1-introduction",
    "offset": 0
  }'
```

#### Chat Query (Regular mode)
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is artificial intelligence?",
    "session_id": "session-123"
  }'
```

#### Chat Query (Selected-text-only mode)
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Explain this concept in more detail",
    "selected_text": "Artificial intelligence is a wonderful field that combines computer science and cognitive abilities...",
    "session_id": "session-123"
  }'
```

### Using Python requests

```python
import requests
import json

BASE_URL = "http://localhost:8000"

# Embed text
embed_data = {
    "text": "Artificial intelligence is a wonderful field...",
    "chapter_id": "chapter-1-introduction",
    "offset": 0
}

response = requests.post(f"{BASE_URL}/embed", json=embed_data)
print(response.json())

# Chat query
chat_data = {
    "question": "What is artificial intelligence?",
    "session_id": "session-123"
}

response = requests.post(f"{BASE_URL}/chat", json=chat_data)
print(response.json())
```

## Environment Variables
Create a `.env` file in the project root with the following variables:

```env
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key
NEON_DATABASE_URL=your_neon_database_url
GEMINI_API_KEY=your_gemini_api_key
```

## Running the API

1. Install dependencies:
```bash
cd backend
uv venv
source .venv/Scripts/activate  # On Windows use: .venv\Scripts\activate
pip install -r requirements.txt
```

2. Set up environment variables in `.env` file

3. Run the server:
```bash
cd backend
python main.py
```

Or using uvicorn:
```bash
cd backend
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

## Database Schema

The API uses PostgreSQL with the following tables:

### sessions
- `session_id`: VARCHAR(255) - Primary key
- `user_id`: VARCHAR(255) - Optional user identifier
- `created_at`: TIMESTAMP - Creation timestamp (default: current timestamp)

### questions
- `question_id`: SERIAL - Primary key
- `session_id`: VARCHAR(255) - Foreign key referencing sessions
- `question`: TEXT - The user's question
- `answer`: TEXT - The AI-generated answer
- `timestamp`: TIMESTAMP - Creation timestamp (default: current timestamp)
```

## Error Handling

- `422 Unprocessable Entity`: Invalid request body format
- `500 Internal Server Error`: Server error during processing
- Error responses follow the format:
```json
{
  "detail": "Error message describing the issue"
}
```