# Quickstart: Integrated RAG Chatbot MVP

**Branch**: `002-gemini-rag-chatbot` | **Date**: 2025-12-16 | **Plan**: [plan.md](plan.md)

This guide provides the essential steps to get the RAG Chatbot MVP running locally. This setup is based on the ultra-fast, 2-day build plan.

---

### Prerequisites

1.  **Python 3.11+** and `pip`.
2.  **Node.js LTS** and `npm`/`yarn`.
3.  **Qdrant Cloud Account**: Sign up for a free tier account at [cloud.qdrant.io](https://cloud.qdrant.io/).
4.  **Google AI API Key**: Obtain an API key for Gemini from [Google AI Studio](https://aistudio.google.com/app/apikey).
5.  **Repository Access**: Clone the `ai_book` repository.

---

### Step 1: Backend Setup

1.  **Navigate to the backend directory**:
    ```bash
    cd backend
    ```

2.  **Create a virtual environment and install dependencies**:
    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows use `venv\Scripts\activate`
    pip install -r requirements.txt 
    ```
    *(Note: `requirements.txt` will need to be created with `fastapi`, `uvicorn`, `openai`, `qdrant-client`, etc.)*

3.  **Configure Environment Variables**:
    Create a `.env` file in the `backend` directory with the following:
    ```env
    # .env
    GOOGLE_API_KEY="your-gemini-api-key"
    QDRANT_URL="your-qdrant-cloud-url"
    QDRANT_API_KEY="your-qdrant-api-key"
    QDRANT_COLLECTION_NAME="ai_book_rag"
    ```

4.  **Run the Backend Server**:
    ```bash
    uvicorn src.main:app --reload
    ```
    The API will be available at `http://127.0.0.1:8000`.

---

### Step 2: Data Ingestion

This is a one-time manual step for the MVP.

1.  **Navigate to the scripts directory**:
    ```bash
    cd ../scripts
    ```

2.  **Ensure dependencies are installed**:
    The ingestion script will have its own dependencies like `beautifulsoup4`, `langchain`, etc. Install them as needed.

3.  **Run the ingestion script**:
    The script will be configured to use the same `.env` variables as the backend.
    ```bash
    python ingest.py
    ```
    This script will:
    -   Load the Docusaurus book content (assuming a local build exists or it scrapes the live site).
    -   Chunk the text.
    -   Generate embeddings using the Gemini API.
    -   Upsert the vectors and payloads to your Qdrant collection.

---

### Step 3: Frontend Setup

1.  **Navigate to the Docusaurus project root**:
    ```bash
    cd .. 
    ```

2.  **Inject the Chat Widget**:
    The plan involves injecting a custom React component. For the quickstart, this will likely involve adding a `<script>` tag or custom JavaScript file via Docusaurus configuration (`docusaurus.config.js`). The exact mechanism will be defined in the tasks.

3.  **Start the Docusaurus development server**:
    ```bash
    npm run start
    ```
    The book will be available at `http://localhost:3000`. The chat widget should appear in the bottom-right corner, connected to your local backend.

---

### Validation

-   Open the Docusaurus site at `http://localhost:3000`.
-   Open the chat widget.
-   Ask a question that can be answered by the book content (e.g., "What is URDF?").
-   Verify that you receive a coherent, grounded answer in under a few seconds.
