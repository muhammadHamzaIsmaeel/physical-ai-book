# backend/src/services/rag_service.py
import os
from typing import List, Dict, Any, cast
from openai import OpenAI
from sqlmodel import select, Session
from uuid import UUID

from ..core.qdrant import get_qdrant_client
from ..core.config import get_cached_qdrant_config, get_cached_gemini_client
from ..database.database import engine
from ..database.models import DocumentChunkMetadata

from dotenv import load_dotenv
load_dotenv()


async def retrieve_context(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Retrieves the top_k most relevant document chunks.
    Queries Qdrant for vector IDs, then fetches full metadata from Neon Postgres.
    """
    # 1. Get the embedding for the user's query
    gemini_api_key = os.getenv("GOOGLE_API_KEY")
    if not gemini_api_key:
        raise ValueError("GOOGLE_API_KEY not found in environment variables")

    # Use a synchronous client for embedding
    sync_gemini_client = OpenAI(
        api_key=gemini_api_key,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
    )

    try:
        query_embedding_response = sync_gemini_client.embeddings.create(
            input=[query],
            model="text-embedding-004"
        )
        query_vector = query_embedding_response.data[0].embedding
    except Exception as e:
        print(f"Error generating embedding: {e}")
        raise

    # Get collection name and validate it's not None
    qdrant_config = get_cached_qdrant_config()
    collection_name = qdrant_config["collection_name"]
    if not collection_name:
        raise ValueError("QDRANT_COLLECTION_NAME is not set")

    # Cast to str to satisfy type checker
    collection_name_str = cast(str, collection_name)

    # 2. Query Qdrant for similar vectors to get their IDs
    search_results: Any = None
    qdrant_client = get_qdrant_client()

    try:
        # Try newer API first (query_points)
        search_results = qdrant_client.query_points(  # type: ignore
            collection_name=collection_name_str,
            query=query_vector,
            limit=top_k,
            with_payload=True
        )
    except (AttributeError, TypeError):
        # Fallback to older API (search)
        try:
            search_results = qdrant_client.search(  # type: ignore
                collection_name=collection_name_str,
                query_vector=query_vector,
                limit=top_k,
                with_payload=True
            )
        except Exception as e:
            print(f"Error searching Qdrant: {e}")
            raise
    except Exception as e:
        print(f"Error querying Qdrant: {e}")
        raise

    # Extract postgres IDs from search results
    postgres_ids: List[str] = []

    # Handle both query_points response (ScoredPoint) and search response
    if hasattr(search_results, 'points'):
        # query_points response - has .points attribute
        for hit in search_results.points:  # type: ignore
            if hasattr(hit, 'payload') and hit.payload and "postgres_id" in hit.payload:
                postgres_ids.append(str(hit.payload["postgres_id"]))
    elif isinstance(search_results, list):
        # search response - is a list directly
        for hit in search_results:
            if hasattr(hit, 'payload') and hit.payload and "postgres_id" in hit.payload:
                postgres_ids.append(str(hit.payload["postgres_id"]))

    # 3. Fetch full metadata from Neon Postgres
    context_from_db: List[Dict[str, Any]] = []

    if postgres_ids:
        # Convert string IDs to UUID objects
        uuid_ids: List[UUID] = [UUID(pid) for pid in postgres_ids]

        with Session(engine) as session:
            # Use proper SQLAlchemy syntax for IN clause
            # The .in_ method is from SQLAlchemy ColumnElement
            statement = select(DocumentChunkMetadata).where(
                DocumentChunkMetadata.id.in_(uuid_ids)  # type: ignore[attr-defined]
            )
            db_results = session.exec(statement).all()

            # Create a map for efficient lookup
            db_results_map: Dict[str, DocumentChunkMetadata] = {
                str(item.id): item for item in db_results
            }

            # Reorder results to match Qdrant's relevance order
            if hasattr(search_results, 'points'):
                # query_points response
                for hit in search_results.points:  # type: ignore
                    if hasattr(hit, 'payload') and hit.payload and "postgres_id" in hit.payload:
                        postgres_id = str(hit.payload["postgres_id"])
                        db_item = db_results_map.get(postgres_id)
                        if db_item:
                            context_from_db.append({
                                "content": db_item.content,
                                "source_url": db_item.source_url,
                                "source_title": db_item.source_title,
                                "score": getattr(hit, 'score', 0.0)
                            })
            else:
                # search response - list of ScoredPoint
                for hit in search_results:
                    if hasattr(hit, 'payload') and hit.payload and "postgres_id" in hit.payload:
                        postgres_id = str(hit.payload["postgres_id"])
                        db_item = db_results_map.get(postgres_id)
                        if db_item:
                            context_from_db.append({
                                "content": db_item.content,
                                "source_url": db_item.source_url,
                                "source_title": db_item.source_title,
                                "score": getattr(hit, 'score', 0.0)
                            })

    return context_from_db


async def generate_answer(
    query: str,
    context: List[Dict[str, Any]],
    history: List[Dict[str, str]]
):
    """
    Generates an answer using the Gemini model based on the query, retrieved context, and chat history.
    """
    system_prompt = """
You are an expert assistant for the 'Physical AI & Humanoid Robotics Course Book'.
Your role is to answer user questions based *only* on the provided context.
Do not use any external knowledge.
If the context does not contain the answer, state that you cannot answer based on the provided information.
Format your response clearly using Markdown.
After the answer, list the sources you used from the context with their URLs.
"""

    # Format context
    context_str = "\n\n---\n\n".join([
        f"Source: {item.get('source_title', 'N/A')} ({item.get('source_url', 'N/A')})\nContent: {item['content']}"
        for item in context
    ])

    user_message = f"""
Based on the following context, please answer the user's question.

CONTEXT:
---
{context_str}
---

USER QUESTION:
{query}
"""

    # Build messages list with proper typing
    messages: List[Dict[str, str]] = [
        {"role": "system", "content": system_prompt},
    ]

    # Add history if it exists
    if history:
        messages.extend(history)

    messages.append({"role": "user", "content": user_message})

    try:
        # Use gemini_client with proper parameters
        gemini_client = get_cached_gemini_client()
        stream = await gemini_client.chat.completions.create(
            model="gemini-2.5-flash",
            messages=messages,  # type: ignore[arg-type]
            stream=True,
        )
        return stream
    except Exception as e:
        print(f"Error generating answer: {e}")
        raise
