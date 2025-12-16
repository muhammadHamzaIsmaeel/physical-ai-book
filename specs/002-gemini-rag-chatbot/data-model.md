# Data Model: Integrated RAG Chatbot

**Branch**: `002-gemini-rag-chatbot` | **Date**: 2025-12-16 | **Plan**: [plan.md](plan.md)

This document defines the data structures used by the RAG chatbot feature. The data model now incorporates both Neon Serverless Postgres for storing detailed document metadata and Qdrant for managing vector embeddings, linking them via a common ID.

---

### Entity: Document Chunk Metadata (Stored in Neon Postgres)

This entity represents the textual content and associated metadata of a document chunk, which is primarily stored in a relational database.

- **Description**: A `DocumentChunkMetadata` is a segment of text from the source content, along with its original location information.
- **Storage**: Neon Serverless Postgres (table: `document_chunks`)

- **Fields**:
  - **`id`** (UUID or Serial Primary Key): A unique identifier for this chunk in the Postgres database. This ID will also be used as the point ID in Qdrant.
  - **`content`** (TEXT, required): The actual text content of the chunk.
  - **`source_url`** (TEXT, required): The URL of the book page from which the chunk was extracted.
  - **`source_title`** (TEXT, optional): The title of the page or section.
  - **`created_at`** (TIMESTAMP, default: NOW()): Timestamp of when the chunk was added.
  - **`updated_at`** (TIMESTAMP, default: NOW()): Timestamp of the last update.

### Entity: Document Chunk Vector (Stored in Qdrant Cloud)

This entity represents the vector embedding of a document chunk, stored in a vector database and linked to its metadata in Postgres.

- **Description**: A `DocumentChunkVector` is the numerical representation (embedding) of a `DocumentChunkMetadata`'s content.
- **Storage**: Qdrant Cloud Collection

- **Fields**:
  - **`id`** (UUID): Matches the `id` of the corresponding `DocumentChunkMetadata` in Postgres.
  - **`vector`** (float array): The Gemini-generated embedding for the `content` from Postgres.
  - **`payload`** (object, optional): Minimal payload in Qdrant, typically just `postgres_id` or `source_url` for quick lookup, though the primary metadata resides in Postgres.

### Relationships

- **One-to-One**: There is a one-to-one relationship between a `DocumentChunkMetadata` record in Neon Postgres and a `DocumentChunkVector` point in Qdrant, linked by their shared `id`.

### State Transitions

Entities are largely static once created. Updates to book content will involve:
1.  Deleting old chunks (both from Postgres and Qdrant).
2.  Creating new chunks (inserting into Postgres, then embedding and upserting into Qdrant with the Postgres ID).
