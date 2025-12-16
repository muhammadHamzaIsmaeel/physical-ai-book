# backend/src/api/chat.py
from fastapi import APIRouter
from pydantic import BaseModel
from typing import List, Optional
from starlette.responses import StreamingResponse
from ..services import rag_service
import json

router = APIRouter()

class ChatMessage(BaseModel):
    role: str
    content: str

class ChatRequest(BaseModel):
    message: str
    history: Optional[List[ChatMessage]] = None

async def stream_generator(query: str, history: list = []):
    """
    A generator function that yields parts of the response stream.
    """
    # 1. Retrieve context
    context = await rag_service.retrieve_context(query)

    # 2. Generate a streaming answer
    response_stream = await rag_service.generate_answer(query, context, history)

    async for chunk in response_stream:
        content = chunk.choices[0].delta.content or ""
        # Use proper JSON encoding to avoid escaping issues
        yield f"data: {json.dumps({'content': content})}\n\n"

    # Signal the end of the stream
    yield "data: [DONE]\n\n"

@router.post("/chat")
async def chat_endpoint(request: ChatRequest):
    """
    Handles the chat request, retrieves context, and streams back the response.
    """
    # Convert Pydantic models to dictionaries for the service layer
    history_dicts = [msg.dict() for msg in request.history] if request.history else []
    
    return StreamingResponse(
        stream_generator(request.message, history_dicts),
        media_type="text/event-stream"
    )
