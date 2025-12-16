# Implementation Plan: Integrated RAG Chatbot

**Branch**: `002-gemini-rag-chatbot` | **Date**: 2025-12-16 | **Spec**: [specs/002-gemini-rag-chatbot/spec.md](specs/002-gemini-rag-chatbot/spec.md)
**Input**: Feature specification from `specs/002-gemini-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command.

## Summary

This plan outlines an ultra-fast, 2-day build for an integrated RAG (Retrieval-Augmented Generation) chatbot for the Physical AI & Humanoid Robotics course book. The chatbot will be powered by Google's Gemini 1.5 Flash model, accessed via the OpenAI-compatible SDK, ensuring high performance and low latency. The architecture consists of a minimal FastAPI backend, a Qdrant vector store, and a custom React chat widget embedded in the Docusaurus site. The primary goal is to deliver a functional MVP that provides accurate, grounded answers from the book's content within the aggressive timeline.

## Technical Context

**Language/Version**: Python 3.11+ (Backend), TypeScript/React (Frontend)
**Primary Dependencies**: FastAPI, OpenAI Agents SDK, Qdrant-client, psycopg2-binary, Docusaurus, React
**Storage**: Qdrant Cloud (Free Tier) for vector embeddings. Neon Serverless Postgres (Free Tier) with `pgvector` for document chunk metadata (content, source_url, source_title, etc.).
**Testing**: Pytest (for backend smoke tests), manual browser testing (for frontend and E2E)
**Target Platform**: Web (Docusaurus site on GitHub Pages, backend on Render/Vercel)
**Project Type**: Web application (frontend/backend)
**Performance Goals**: <2 second end-to-end response time for 95% of queries.
**Constraints**: Must operate within free tiers of all services.
**Scale/Scope**: Initial scope is a single chatbot instance for the course book, with a focus on core Q&A functionality. Selected-text queries and automated ingestion are deferred post-MVP.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Spec-First Development**: PASS. This plan is derived directly from an approved spec.
- **AI-Native Workflow**: PASS. The core of the feature is an AI-native RAG pipeline.
- **Maximum Truth-Seeking**: PASS. The entire architecture is designed to provide grounded, cited answers and minimize hallucination.
- **Progressive Enhancement**: MINOR DEVIATION (JUSTIFIED). The chat widget will require JavaScript to function. This is an acceptable tradeoff as the core book content remains accessible without JS, and the widget is a clear enhancement.
- **Open-Source Excellence**: PASS. The plan relies exclusively on open-source technologies and services with free tiers.
- **Accessibility & Standards**: CONDITIONAL PASS. The implementation of the React chat widget must adhere to WCAG 2.1 AA standards. This will be a requirement in the tasks.
- **Performance & Constraints**: PASS. The plan explicitly targets aggressive performance goals and operates within a 2-day constraint.

## Project Structure

### Documentation (this feature)

```text
specs/002-gemini-rag-chatbot/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── openapi.yml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Application Structure
src/
├── components/
│   └── ChatWidget/      # React chat widget component
└── theme/               # Docusaurus swizzled components for integration
... (existing docusaurus structure)

backend/
├── src/
│   ├── main.py          # FastAPI app entrypoint
│   ├── core/            # Gemini client config, settings
│   ├── services/        # RAG agent logic, Qdrant service
│   ├── api/             # API endpoint definitions (/chat)
│   └── database/        # Database schema and utilities (Neon Postgres)
└── tests/
    └── test_chat.py     # Smoke tests for the /chat endpoint

scripts/
└── ingest.py            # Manual ingestion script
```

**Structure Decision**: The project will consist of the existing Docusaurus application (`src/`), a new `backend/` service for the chatbot API, and a `scripts/` directory for data ingestion. The chat widget will be developed as a new React component inside `src/components/ChatWidget` to integrate directly with the existing Docusaurus frontend. The backend will utilize Neon Serverless Postgres for metadata storage, with database-related code residing in `backend/src/database/`. This updated structure supports both Qdrant for vectors and Neon for metadata.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| JavaScript Requirement for Chat Widget | The chat widget is an interactive application that fundamentally requires JavaScript for its operation. | A non-JS version (e.g., a simple form post) was rejected because it would not support the required real-time, conversational user experience (streaming responses, session history). |
