# main.py - FastAPI app entrypoint
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .api import chat
from .database.database import create_db_and_tables

app = FastAPI(
    title="RAG Chatbot API",
    version="1.0.0",
    on_startup=[create_db_and_tables] # Call this function on application startup
)

# Add CORS middleware to allow frontend requests
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Docusaurus dev server
        "http://localhost:3001",
        "http://127.0.0.1:3000",
        "https://muhammadhamzaismaeel.github.io",  # GitHub Pages (production)
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include the chat router
app.include_router(chat.router)

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running."}
