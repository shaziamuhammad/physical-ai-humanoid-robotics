# RAG Chatbot Backend

This backend provides a Retrieval-Augmented Generation (RAG) chatbot for the Physical AI & Humanoid Robotics book. It uses Cohere for embeddings, Qdrant for vector storage, and Google Gemini for generating responses.

## Features

- **Text Embedding**: Convert book content into vector embeddings using Cohere
- **Semantic Search**: Find relevant content using vector similarity
- **RAG Responses**: Generate answers based on book content with citations
- **Selected-text-only Mode**: Answer questions based only on user-selected text
- **Session Management**: Track conversation history
- **Database Storage**: Store questions, answers, and sessions in PostgreSQL

## Prerequisites

- Python 3.8+
- uv package manager
- API keys for:
  - Cohere
  - Google Gemini
  - Qdrant (if using cloud)
- PostgreSQL database (Neon recommended)

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd physical-ai-humanoid-robotics
   ```

2. **Navigate to backend directory**
   ```bash
   cd backend
   ```

3. **Create and set up environment variables**
   ```bash
   cp ../.env .env  # If you have a .env file in the root
   # Or create a new .env file with your API keys
   ```

4. **Install dependencies**
   ```bash
   uv venv
   source .venv/Scripts/activate  # On Windows: .venv\Scripts\activate
   uv pip install -r requirements.txt
   ```

## Configuration

Create a `.env` file in the backend directory (or use the one in the root) with the following variables:

```env
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key
NEON_DATABASE_URL=your_neon_database_url
GEMINI_API_KEY=your_gemini_api_key
```

## Running the Server

### Method 1: Using the run script (Windows)
```bash
./run_server.bat
```

### Method 2: Manual start
```bash
# Activate virtual environment
source .venv/Scripts/activate  # On Windows: .venv\Scripts\activate

# Run the server
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

The server will be available at `http://localhost:8000`

## API Endpoints

See [API Documentation](API_DOCUMENTATION.md) for full details.

- `GET /` - Health check
- `POST /embed` - Embed and store text content
- `POST /chat` - Query the RAG system

## Testing

### Test the health endpoint
```bash
curl http://localhost:8000/
```

### Test embedding
```bash
curl -X POST http://localhost:8000/embed \
  -H "Content-Type: application/json" \
  -d '{
    "text": "Artificial intelligence is a wonderful field...",
    "chapter_id": "test-chapter",
    "offset": 0
  }'
```

### Test chat
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is artificial intelligence?",
    "session_id": "test-session"
  }'
```

## Frontend Integration

The backend is designed to work with the Docusaurus frontend. The chatbot component is automatically added to all pages through the custom theme layout.

## Architecture

- **Embeddings**: Cohere's multilingual embedding model
- **Vector Storage**: Qdrant vector database
- **Generation**: Google Gemini 1.0 Pro
- **Database**: PostgreSQL (Neon)
- **Framework**: FastAPI
- **Tokenization**: TikToken for chunking

## Troubleshooting

1. **API Keys**: Ensure all required API keys are set in the `.env` file
2. **Database Connection**: Verify the PostgreSQL connection string is correct
3. **Vector Database**: Ensure Qdrant is accessible and credentials are correct
4. **CORS**: The server allows all origins by default - update for production

## Development

For development, use the `--reload` flag to automatically restart the server when code changes:

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

## Deployment

The backend can be deployed to any platform that supports Python applications (Vercel, Render, AWS, etc.). Make sure to set the required environment variables in the deployment environment.