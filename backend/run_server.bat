@echo off
REM Script to run the RAG Chatbot backend

REM Activate virtual environment
if exist ".venv" (
    echo Activating virtual environment...
    call .venv\Scripts\activate.bat
) else (
    echo Creating virtual environment...
    uv venv
    call .venv\Scripts\activate.bat
    echo Installing dependencies...
    pip install -r requirements.txt
)

REM Run the FastAPI server
echo Starting RAG Chatbot API server...
uvicorn main:app --reload --host 0.0.0.0 --port 8000