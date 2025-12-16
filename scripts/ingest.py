# scripts/ingest.py

import argparse
import os
from pathlib import Path
from dotenv import load_dotenv
import sys
from typing import List

# Add backend.src to sys.path to allow imports from backend.src when script is run from root
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'backend', 'src'))

# Import Langchain components - using TextLoader instead of UnstructuredMarkdownLoader
from langchain_community.document_loaders import TextLoader
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_core.documents import Document

# Import database components
# Note: These imports work at runtime due to sys.path manipulation above
import database.database  # type: ignore
import database.models  # type: ignore
from sqlmodel import Session

# Import Qdrant and OpenAI client for embeddings
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance
from openai import OpenAI


def load_markdown_files(docs_path: str) -> List[Document]:
    """
    Load all markdown files from the docs directory.
    Uses simple TextLoader instead of UnstructuredMarkdownLoader to avoid dependencies.
    """
    documents: List[Document] = []
    docs_dir = Path(docs_path)

    if not docs_dir.exists():
        raise ValueError(f"Documentation path does not exist: {docs_path}")

    # Find all .md and .mdx files
    markdown_files = list(docs_dir.glob("**/*.md")) + list(docs_dir.glob("**/*.mdx"))

    print(f"Found {len(markdown_files)} markdown files")

    for file_path in markdown_files:
        try:
            # Read file content directly
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Create a Document object
            relative_path = file_path.relative_to(docs_dir.parent)
            doc = Document(
                page_content=content,
                metadata={
                    "source": str(relative_path),
                    "title": file_path.stem.replace('-', ' ').title()
                }
            )
            documents.append(doc)

        except Exception as e:
            print(f"Warning: Could not load {file_path}: {e}")
            continue

    print(f"Successfully loaded {len(documents)} documents")
    return documents


def main():
    """
    Main function to run the data ingestion process.
    This script will:
    1. Load the Docusaurus book content.
    2. Chunk the text.
    3. Generate embeddings using the Gemini API.
    4. Upsert the vectors and payloads to the Qdrant collection.
    """
    parser = argparse.ArgumentParser(description="Ingest data into Qdrant for the RAG chatbot.")
    parser.add_argument("--path", type=str, default="./", help="Path to the Docusaurus project root.")
    args = parser.parse_args()

    # Load environment variables from backend/.env
    dotenv_path = os.path.join(os.path.dirname(__file__), '..', 'backend', '.env')
    load_dotenv(dotenv_path=dotenv_path)

    print("Starting data ingestion...")
    print(f"Loading data from: {args.path}")

    # 1. Load the Docusaurus book content
    docs_path = os.path.join(args.path, "docs")
    documents = load_markdown_files(docs_path)

    if not documents:
        raise ValueError("No documents loaded. Check if the docs/ directory exists and contains markdown files.")

    # 2. Chunk the text
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=1000,
        chunk_overlap=200,
    )

    chunked_documents = text_splitter.split_documents(documents)
    print(f"Split documents into {len(chunked_documents)} chunks.")

    # Database setup (for Postgres)
    print("Setting up database tables...")
    database.database.create_db_and_tables()

    # 3. Generate embeddings and 4. Upsert to Qdrant and Postgres
    gemini_api_key = os.getenv("GOOGLE_API_KEY")
    if not gemini_api_key:
        raise ValueError("GOOGLE_API_KEY not found in environment variables. Please set it in backend/.env")

    # Use OpenAI client with Gemini's OpenAI-compatible endpoint
    embeddings_client = OpenAI(
        api_key=gemini_api_key,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
    )

    # Get Qdrant configuration with proper None handling
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    collection_name = os.getenv("QDRANT_COLLECTION_NAME")

    # Validate required environment variables
    if not qdrant_url:
        raise ValueError("QDRANT_URL not set in environment variables")
    if not collection_name:
        raise ValueError("QDRANT_COLLECTION_NAME not set in environment variables")

    # Initialize Qdrant client
    qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

    # Ensure Qdrant collection exists with correct configuration
    # Gemini's text-embedding-004 produces 768-dimensional vectors
    vector_size = 768

    try:
        # Use proper VectorParams type instead of dict
        vector_config = VectorParams(
            size=vector_size,
            distance=Distance.COSINE
        )

        qdrant_client.recreate_collection(
            collection_name=collection_name,
            vectors_config=vector_config
        )
        print(f"✓ Recreated Qdrant collection '{collection_name}' with vector size {vector_size}")
    except Exception as e:
        print(f"Warning: Could not recreate collection: {e}")
        print("Attempting to use existing collection...")
        try:
            qdrant_client.get_collection(collection_name=collection_name)
            print(f"✓ Using existing collection '{collection_name}'")
        except Exception as inner_e:
            raise ValueError(f"Collection '{collection_name}' does not exist and could not be created: {inner_e}")

    print(f"\nProcessing {len(chunked_documents)} chunks...")
    print("This may take a while depending on the number of chunks and API rate limits.\n")

    points = []
    successful_chunks = 0

    with Session(database.database.engine) as session:
        for i, doc_chunk in enumerate(chunked_documents):
            try:
                # 1. Insert metadata into Postgres
                source_path = doc_chunk.metadata.get("source", "N/A")
                source_title = doc_chunk.metadata.get("title",
                    source_path.split('/')[-1].replace('.mdx', '').replace('.md', '')
                )

                metadata = database.models.DocumentChunkMetadata(
                    content=doc_chunk.page_content,
                    source_url=source_path,
                    source_title=source_title
                )
                session.add(metadata)
                session.commit()
                session.refresh(metadata)

                # 2. Generate embedding for the content using OpenAI client
                embedding_response = embeddings_client.embeddings.create(
                    input=[doc_chunk.page_content],
                    model="text-embedding-004"
                )
                embedding = embedding_response.data[0].embedding

                # 3. Prepare point for Qdrant
                points.append(
                    PointStruct(
                        id=str(metadata.id),
                        vector=embedding,
                        payload={
                            "postgres_id": str(metadata.id),
                            "source_url": source_path,
                            "source_title": source_title
                        }
                    )
                )

                successful_chunks += 1

                # Batch upsert every 50 chunks to avoid memory issues
                if (i + 1) % 50 == 0:
                    qdrant_client.upsert(
                        collection_name=collection_name,
                        wait=True,
                        points=points
                    )
                    print(f"Progress: {i + 1}/{len(chunked_documents)} chunks processed and upserted")
                    points = []

            except Exception as e:
                print(f"Error processing chunk {i + 1}: {e}")
                continue

        # Upsert any remaining points
        if points:
            qdrant_client.upsert(
                collection_name=collection_name,
                wait=True,
                points=points
            )
            print(f"Progress: {len(chunked_documents)}/{len(chunked_documents)} chunks processed")

    print(f"\n✓ Data ingestion complete!")
    print(f"  - Successfully processed: {successful_chunks}/{len(chunked_documents)} chunks")
    print(f"  - Stored in Postgres: {successful_chunks} records")
    print(f"  - Stored in Qdrant: {successful_chunks} vectors")


if __name__ == "__main__":
    main()
