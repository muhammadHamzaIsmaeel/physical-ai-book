# backend/src/database/models.py
from typing import Optional
from uuid import UUID, uuid4
from datetime import datetime
from sqlmodel import Field, SQLModel, create_engine

class DocumentChunkMetadata(SQLModel, table=True):
    id: Optional[UUID] = Field(default_factory=uuid4, primary_key=True, index=True)
    content: str
    source_url: str
    source_title: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow, nullable=False)
    updated_at: datetime = Field(default_factory=datetime.utcnow, nullable=False)

    class Config:
        json_encoders = {
            UUID: str,
            datetime: lambda dt: dt.isoformat()
        }

# Ensure SQLModel is imported so `SQLModel.metadata` is populated
# This is usually done by importing models that inherit from SQLModel.
# For example: from .models import DocumentChunkMetadata
