from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import os
from typing import Optional, List
from dotenv import load_dotenv
from embedding import embed_and_store, initialize_collection
from chat import generate_response
from db import db_manager

# Load environment variables
load_dotenv()

app = FastAPI(title="RAG Chatbot API", version="1.0.0")

# Enable CORS for Docusaurus frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize database tables on startup
@app.on_event("startup")
async def startup_event():
    await db_manager.initialize_tables()
    initialize_collection()

# Request/Response models
class EmbedRequest(BaseModel):
    text: str
    chapter_id: str
    offset: int = 0

class ChatRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    citations: List[dict]
    session_id: str

@app.get("/")
def health_check():
    return {"status": "healthy", "service": "RAG Chatbot API"}

@app.post("/embed")
async def embed_text(request: EmbedRequest):
    """Endpoint to embed text and store in vector database"""
    try:
        result = embed_and_store(request.text, request.chapter_id, request.offset)
        return result
    except ValueError as ve:
        raise HTTPException(status_code=422, detail=f"Validation error: {str(ve)}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing embeddings: {str(e)}")

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """Endpoint to handle chat queries using RAG"""
    try:
        result = await generate_response(request.question, request.selected_text, request.session_id)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)