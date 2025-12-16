# Feature Specification: Integrated RAG Chatbot for Physical AI & Humanoid Robotics Course Book

**Feature Branch**: `002-gemini-rag-chatbot`  
**Created**: 2025-12-16
**Status**: Draft  
**Input**: User description: "Integrated RAG Chatbot for Physical AI & Humanoid Robotics Course Book (Gemini-Powered via OpenAI Agents SDK) Target audience: Students, educators, and developers accessing the online course book on Physical AI and Humanoid Robotics Focus: Enabling accurate, context-aware querying of the book's content with Retrieval-Augmented Generation, including support for questions on user-selected text. Leverage Google's Gemini models via the OpenAI-compatible endpoint for cost-effective, high-performance generation and embeddings. Success criteria: - Chatbot accurately answers questions based solely on the book's content from the Docusaurus site - Supports retrieval from the full book text and from user-highlighted/selected sections - Provides cited responses linking back to relevant book sections or pages - Handles conversational context across multiple turns - Embedded seamlessly in the published Docusaurus book with a user-friendly chat interface - All responses grounded in retrieved content, minimizing hallucinations - Successfully routes OpenAI Agents/ChatKit SDK to Gemini models using the gemini-via-openai-agents skill configuration Constraints: - Technologies: FastAPI backend, OpenAI Agents/ChatKit SDKs for agentic RAG workflow, Neon Serverless Postgres (with pgvector for metadata/storage if needed), Qdrant Cloud Free Tier for vector embeddings and retrieval - Embeddings: Use Gemini embedding models (e.g., gemini-embedding-001) via OpenAI-compatible endpoint - Generation: Use Gemini models (e.g., gemini-2.5-flash or gemini-2.5-pro) via OpenAI-compatible endpoint - Configuration: Follow the gemini-via-openai-agents skill (located at .gemini/skills/gemini-via-openai-agents/SKILL.md) to create custom AsyncOpenAI client with Gemini API key and base_url='https://generativelanguage.googleapis.com/v1beta/openai/' - Frontend integration: Embed chat widget/interface directly into Docusaurus pages (e.g., fixed widget, dedicated chat page, or iframe) - Deployment: Backend deployed separately (e.g., Vercel, Render, or similar); book remains on GitHub Pages - Data ingestion: Automated or scripted process to chunk book Markdown/HTML, generate embeddings using Gemini via OpenAI client, and upsert into Qdrant upon book updates - Timeline: Complete within 2 days Not building: - Full search replacement for Docusaurus (keep existing search if any) - General-purpose chatbot (strictly grounded in book content only) - Multi-modal features beyond Gemini's native support (e.g., no forced voice input/output) - User authentication or personalized history beyond session - Advanced analytics dashboard or admin panel - On-device/local inference (cloud-based services only) - Direct use of native Google GenAI SDK (must route through OpenAI Agents SDK via Gemini compatibility layer)"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Query the book for information (Priority: P1)

A student reading the course book has a question about a concept. They open the chat interface, type their question, and receive a concise, accurate answer based on the book's content, along with a link to the relevant section for further reading.

**Why this priority**: This is the core functionality of the feature. It directly addresses the primary user need of getting quick, contextual answers to questions about the course material.

**Independent Test**: Can be fully tested by asking a question that is answerable by the book's content and verifying that the chatbot provides a correct answer with a citation. This delivers immediate value by improving the learning experience.

**Acceptance Scenarios**:

1. **Given** a user is on any page of the Docusaurus book, **When** they ask the chatbot "What is URDF?", **Then** the chatbot provides a correct definition based on the book's content and a link to the "URDF Basics" page.
2. **Given** a user asks a question whose answer is not in the book, **When** they submit the query, **Then** the chatbot responds that it cannot answer the question based on the available content.

---

### User Story 2 - Ask a question about selected text (Priority: P2)

An educator is preparing a lesson and highlights a specific paragraph in the book. They use the chatbot to ask a clarifying question about that highlighted text. The chatbot provides an answer specifically contextualized to the selected passage.

**Why this priority**: This enhances the core query functionality by allowing for more focused and nuanced questions, which is a significant value-add for users doing in-depth analysis.

**Independent Test**: Can be tested by highlighting a passage of text, invoking the chatbot, and asking a question relevant only to that passage.

**Acceptance Scenarios**:

1. **Given** a user has highlighted a paragraph about Isaac Sim, **When** they ask the chatbot "What are the key requirements for this?", **Then** the chatbot provides an answer based only on the selected text.

---

### User Story 3 - Ingest updated content (Priority: P3)

A book author updates a chapter with new information. The new content is automatically processed, and its embeddings are added to the vector store, making it available for the chatbot to use in its responses.

**Why this priority**: This ensures the chatbot remains accurate and up-to-date as the book evolves. It's a critical maintenance and reliability function.

**Independent Test**: Can be tested by adding a new markdown file with unique information, triggering the ingestion process, and then asking the chatbot a question that can only be answered using the new information.

**Acceptance Scenarios**:

1. **Given** a new chapter has been added to the book, **When** the ingestion process completes, **Then** questions about the new chapter's content are answered correctly by the chatbot.

---

### Edge Cases

- What happens when a user asks a question in a language other than English?
- How does the system handle extremely long user queries or very large selected text passages?
- What is the expected response when the query is ambiguous or nonsensical?
- How does the system respond if the underlying services (Qdrant, Gemini API) are unavailable?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a chat interface for users to ask natural language questions.
- **FR-002**: System MUST retrieve relevant content from the Docusaurus book text to answer questions.
- **FR-003**: System MUST generate answers based on the retrieved content using a Gemini language model.
- **FR-004**: System MUST provide citations/links back to the source document(s) for each answer.
- **FR-005**: System MUST allow users to ask questions about a user-selected (highlighted) passage of text.
- **FR-006**: System MUST maintain conversational context for a user across multiple turns within a single session.
- **FR-007**: System MUST have a process to ingest and create embeddings for new or updated book content.
- **FR-008**: The chat interface MUST be a fixed widget in the bottom-right corner of every Docusaurus page.
- **FR-009**: The ingestion process MUST be automatically triggered via a GitHub Action on pushes to the main branch.
- **FR-010**: The system MUST remember conversational context only for the current user session (clears on page refresh or tab close).

### Key Entities *(include if feature involves data)*

- **Document Chunk**: A piece of text from the book content, stored with its embedding and metadata (e.g., source page, chapter).
- **User Query**: A question asked by the user, including any associated conversational history or selected text.
- **Chatbot Response**: The generated answer, including the text and source citations.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: At least 90% of answers to factual questions drawn from a predefined test set must be rated as "accurate and relevant" by a human evaluator.
- **SC-002**: For queries with answers in the book, the system provides a response with correct citations in under 5 seconds for 95% of requests.
- **SC-003**: The rate of "hallucinated" or factually incorrect answers must be less than 2% in testing.
- **SC-04**: The system must successfully handle a conversation of at least 5 turns, maintaining relevant context throughout.
- **SC-005**: The data ingestion process for the entire book (initial setup) must complete within 1 hour.
