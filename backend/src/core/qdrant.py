# backend/src/core/qdrant.py
from qdrant_client import QdrantClient
from .config import qdrant_config

qdrant_client = QdrantClient(
    url=qdrant_config["url"], 
    api_key=qdrant_config["api_key"]
)
