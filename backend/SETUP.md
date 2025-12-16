# RAG Chatbot Backend Setup

## Prerequisites

- Python 3.9+
- Gemini API key from [Google AI Studio](https://aistudio.google.com/app/apikey)
- Qdrant Cloud account (free tier)
- Neon Postgres database

## Installation

1. **Install Dependencies**

```bash
cd backend
pip install -r requirements.txt
```

2. **Configure Environment Variables**

Copy the example file and fill in your credentials:

```bash
cp .env.example .env
```

Edit `.env` with your actual values:

```env
# Get from: https://aistudio.google.com/app/apikey
GOOGLE_API_KEY=AIzaSy...

# Qdrant Cloud settings
QDRANT_URL=https://your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-key
QDRANT_COLLECTION_NAME=ai_book_embeddings

# Neon Postgres connection string
DATABASE_URL=postgresql://user:password@host/database
```

## Running the Backend

1. **Start the FastAPI server**

```bash
cd backend
uvicorn src.main:app --reload --port 8000
```

2. **Verify it's running**

Visit: http://localhost:8000

You should see: `{"message": "RAG Chatbot API is running."}`

## Data Ingestion

Before the chatbot can answer questions, you need to ingest your book content:

```bash
# From project root
python scripts/ingest.py --path ./
```

This will:
1. Load all markdown files from `docs/`
2. Chunk the text
3. Generate embeddings using Gemini
4. Store in Qdrant + Postgres

## Testing the API

Test the chat endpoint with curl:

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is URDF?",
    "history": []
  }'
```

## Common Issues

### Import Errors

If you see import errors for `langchain_community` or `database.database`:

```bash
pip install langchain-community langchain-openai unstructured
```

### CORS Errors

If frontend can't connect, check that CORS middleware is configured in `src/main.py` with your frontend URL.

### Embedding Dimension Mismatch

Gemini's `text-embedding-004` model produces 768-dimensional vectors. Ensure Qdrant collection is configured correctly:

```python
vector_size = 768
```

### Authentication Errors

Double-check your `GOOGLE_API_KEY` is valid and not expired.

## API Endpoints

- `GET /` - Health check
- `POST /chat` - Chat endpoint (streaming response)

## Next Steps

Once the backend is running:
1. Test with curl/Postman
2. Start the Docusaurus frontend
3. Integrate the ChatWidget component
