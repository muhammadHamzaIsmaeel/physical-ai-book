# Tasks: Integrated RAG Chatbot

**Input**: Design documents from `specs/002-gemini-rag-chatbot/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup

**Purpose**: Project initialization and environment configuration.

- [x] T001 Create backend project structure: `backend/src/main.py`, `backend/src/core/`, `backend/src/services/`, `backend/src/api/`, `backend/tests/`
- [x] T001.1 Create database directory: `backend/src/database/`
- [x] T002 [P] Create ChatWidget component directory in the existing Docusaurus structure: `src/components/ChatWidget/`
- [x] T003 [P] Create scripts directory: `scripts/`
- [x] T004 [P] Initialize backend Python environment and `backend/requirements.txt` with `fastapi`, `uvicorn`, `python-dotenv`, `openai`, `qdrant-client`, `psycopg2-binary`, `sqlmodel`.
- [ ] T005 [P] Ensure Docusaurus development environment is set up by running `npm install`.
- [x] T006 Configure backend environment file `backend/.env` with placeholders for `GOOGLE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`.
- [x] T006.1 Configure backend environment file `backend/.env` with placeholders for `DATABASE_URL` (Neon Postgres).
- [ ] T007 [P] Configure Qdrant Cloud collection with the correct vector size for Gemini embeddings.

---

## Phase 2: Foundational (Backend, Data Ingestion & Neon Postgres)

**Purpose**: Build the data pipeline, integrate Neon Postgres, and establish the core backend service. This phase completes US3 and the backend part of US1.

**⚠️ CRITICAL**: No frontend work can begin until this phase is complete and the `/chat` endpoint is available.

### Database Setup (Neon Postgres)

- [x] T007.1 [P] Implement database connection and session management in `backend/src/database/database.py`.
- [x] T007.2 [P] Define SQLModel for DocumentChunkMetadata in `backend/src/database/models.py`.
- [x] T007.3 [P] Create database tables (run migrations if applicable or direct DDL) for DocumentChunkMetadata.

### Implementation for Data Ingestion (User Story 3)

- [x] T008 [US3] Implement data loading (e.g., from Docusaurus build output) in `scripts/ingest.py`.
- [x] T009 [US3] Implement text chunking logic in `scripts/ingest.py`.
- [x] T010 [US3] Implement Gemini embedding generation for chunks in `scripts/ingest.py`.
- [x] T011 [US3] Implement Qdrant upsert logic to store vectors (referencing Postgres IDs) and insert metadata into Neon Postgres in `scripts/ingest.py`.
- [ ] T012 [US3] Run the full ingestion script `scripts/ingest.py` to populate the Qdrant collection and Neon Postgres.

### Implementation for Backend (User Story 1)

- [x] T013 [P] [US1] Set up Gemini client configuration in `backend/src/core/config.py` using the `gemini-via-openai-agents` skill pattern.
- [x] T014 [P] [US1] Set up Qdrant client and connection in `backend/src/core/qdrant.py`.
- [x] T015 [US1] Implement the core RAG retrieval logic (query Qdrant for IDs, then fetch metadata from Neon Postgres) in `backend/src/services/rag_service.py`.
- [x] T016 [US1] Implement the prompt engineering and Gemini generation logic in `backend/src/services/rag_service.py`.
- [x] T017 [US1] Implement the FastAPI `POST /chat` endpoint in `backend/src/api/chat.py`, ensuring it returns a streaming response.
- [x] T018 [US1] Wire up the chat endpoint and services in the main FastAPI app `backend/src/main.py`.
- [x] T019 [US1] Write a simple smoke test for the `/chat` endpoint in `backend/tests/test_chat.py`.
- [ ] T020 [US1] Manually test the `/chat` endpoint using `curl` or Postman to verify the RAG pipeline works end-to-end.

**Checkpoint**: Backend is functional, data is ingested into both Postgres and Qdrant, and the `/chat` endpoint is serving grounded responses.

---

## Phase 3: User Story 1 - Frontend Chat Widget & Integration

**Goal**: A user can ask a question in a chat widget and receive a streamed response from the backend.

**Independent Test**: The chat widget can be opened on the Docusaurus site, a question can be sent, and a valid, streamed answer is displayed.

### Implementation for Frontend (User Story 1)

- [x] T021 [P] [US1] Build the basic UI for the chat widget (input box, message display area) in `src/components/ChatWidget/ChatWidget.js`.
- [x] T022 [P] [US1] Implement state management for conversation history and loading status in the `ChatWidget` component.
- [x] T023 [US1] Implement the API call to the backend `/chat` endpoint, handling the streaming response (`text/event-stream`).
- [x] T024 [US1] Connect the chat widget to the Docusaurus site, likely via `docusaurus.config.js` or by swizzling a theme component.
- [x] T025 [US1] Manually test the full user flow: open widget, ask question, see loading state, view streamed response, see citations.

**Checkpoint**: The chat widget is integrated and fully functional on the Docusaurus site. The MVP is complete.

---

## Phase 4: Polish & Deployment

**Purpose**: Final touches and making the feature publicly available.

- [x] T026 [P] Refine the chat widget CSS for better styling, responsiveness, and dark/light mode support.
- [x] T027 [P] Add clear error handling and display messages in the UI (e.g., if the backend is down).
- [x] T028 [P] Add loading indicators to the UI to improve user experience.
- [ ] T029 Deploy the FastAPI backend to a free-tier service like Render or Vercel.
- [ ] T030 Update the frontend to point to the deployed backend URL.
- [ ] T031 Merge changes to the main branch to deploy the Docusaurus site with the integrated chat widget.
- [ ] T032 Run final validation on the live site as described in `quickstart.md`.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Must be completed first.
- **Foundational (Phase 2)**: Depends on Setup. This phase is a blocker for the frontend.
- **User Story 1 (Phase 3)**: Depends on Foundational phase completion.
- **Polish (Phase 4)**: Depends on all previous phases.

### User Story Dependencies

- **User Story 3 (Ingestion)**: Must be completed in the Foundational phase before the RAG service can work.
- **User Story 1 (Chat)**: Depends on US3's data. The backend part is in Phase 2, and the frontend part is in Phase 3.
- **User Story 2 (Selected Text)**: Deferred.

### Parallel Opportunities

- **Phase 1**: `T004`, `T005`, and `T007` can be worked on in parallel.
- **Phase 2**: Backend (`T013`-`T018`) and data ingestion (`T008`-`T011`) tasks can be developed in parallel until the final integration test (`T020`).
- **Phase 3**: All frontend tasks can be developed in parallel with the backend phase if a mock API is used, but they depend on the real backend for final integration.

---

## Implementation Strategy

### MVP First (Phases 1-3)

1. Complete **Phase 1 (Setup)**.
2. Complete **Phase 2 (Foundational)**. At this point, the backend is functional and testable via API tools.
3. Complete **Phase 3 (User Story 1)**. At this point, the full MVP is functional and meets the core user need.
4. **STOP and VALIDATE**: Test the complete chat flow from the Docusaurus UI to the backend and back. This is a deployable MVP.
5. Proceed to **Phase 4 (Polish & Deployment)** to make the feature production-ready.
