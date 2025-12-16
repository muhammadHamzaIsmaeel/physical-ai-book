# backend/src/core/config.py
import os
from openai import AsyncOpenAI
from dotenv import load_dotenv
load_dotenv()

def get_gemini_client() -> AsyncOpenAI:
    """
    Returns an AsyncOpenAI client configured for the Gemini API.
    
    This follows the 'gemini-via-openai-agents' skill pattern by using
    the OpenAI library but pointing it to the Google Generative Language API
    endpoint.
    """
    gemini_api_key = os.getenv("GOOGLE_API_KEY")
    if not gemini_api_key:
        raise ValueError("GOOGLE_API_KEY not found in environment variables.")

    # The base_url points to the Gemini API's OpenAI-compatible endpoint
    # NOTE: The trailing slash is required as per gemini-via-openai-agents skill
    return AsyncOpenAI(
        api_key=gemini_api_key,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
    )

def get_qdrant_config():
    """
    Returns a dictionary with Qdrant configuration.
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    collection_name = os.getenv("QDRANT_COLLECTION_NAME")

    if not all([qdrant_url, collection_name]):
        raise ValueError("Qdrant environment variables QDRANT_URL or QDRANT_COLLECTION_NAME not set.")

    return {
        "url": qdrant_url,
        "api_key": qdrant_api_key,
        "collection_name": collection_name
    }

# Instantiate clients or configurations that can be imported elsewhere
gemini_client = get_gemini_client()
qdrant_config = get_qdrant_config()
