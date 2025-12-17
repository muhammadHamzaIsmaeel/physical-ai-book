# backend/src/core/qdrant.py
from qdrant_client import QdrantClient
from .config import get_cached_qdrant_config

# Lazy initializatio
_qdrant_client = None

def get_qdrant_client() -> QdrantClient:
    """Get or create the cached Qdrant client."""
    global _qdrant_client
    if _qdrant_client is None:
        config = get_cached_qdrant_config()
        _qdrant_client = QdrantClient(
            url=config["url"],
            api_key=config["api_key"]
        )
    return _qdrant_client

# For backward compatibility
qdrant_client = None  # Will be initialized on first use
