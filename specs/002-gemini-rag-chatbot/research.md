# Research & Decisions: Integrated RAG Chatbot

**Branch**: `002-gemini-rag-chatbot` | **Date**: 2025-12-16 | **Plan**: [plan.md](plan.md)

This document records the key architectural and technical decisions made during the planning phase for the RAG chatbot, as driven by the ultra-fast 2-day build constraint.

---

### 1. Decision: Chat Interface Embedding Method

- **Decision**: A fixed widget in the bottom-right corner, implemented as a floating chat bubble that expands into a slide-out panel.
- **Rationale**: This approach is the simplest and fastest to implement for a 2-day sprint. It can be built as a self-contained React component and injected into the Docusaurus site via a custom script tag or a simple Docusaurus "swizzle". This avoids complex theme modifications while providing a persistent, easily accessible user experience.
- **Alternatives Considered**:
  - **Dedicated Chat Page**: Rejected as it would require users to navigate away from their reading context.
  - **Iframe Embedding**: Rejected due to potential complexities with styling, cross-origin communication, and responsiveness.

---

### 2. Decision: Data Ingestion Trigger

- **Decision**: A manual, local Python script will be used for the initial data ingestion.
- **Rationale**: The primary goal is to populate the Qdrant vector store as quickly as possible. Building a fully automated GitHub Action is estimated to take 4-6 hours, which is too costly for a 2-day timeline. The manual script achieves the same outcome in a fraction of the time.
- **Alternatives Considered**:
  - **GitHub Actions Automation**: This is the ideal long-term solution but was deferred to post-MVP to save critical development time. It can be added later without impacting the core chatbot functionality.

---

### 3. Decision: Conversational Context Scope

- **Decision**: Conversational context will be managed entirely on the frontend within the React widget's state, lasting only for the current session.
- **Rationale**: This approach results in zero backend complexity. The backend can remain stateless, simplifying its design and deployment. For the MVP's purposes, session-only memory is sufficient to support a useful conversational flow.
- **Alternatives Considered**:
  - **Backend Context Management**: Rejected as it would require user session tracking, a database, and more complex API logic, all of which are out of scope for the 2-day build.
  - **Browser Local Storage**: Rejected as a minor-but-unnecessary complexity for the MVP. Session state is sufficient.

---

### 4. Decision: LLM Backend and Provider

- **Decision**: Google's Gemini 1.5 Flash model will be used, accessed via the OpenAI-compatible endpoint and the `gemini-via-openai-agents` skill configuration.
- **Rationale**: Speed is the most critical factor for a responsive chat experience. Gemini 1.5 Flash is optimized for the lowest latency and is more than capable for this RAG task. The generous free tier is also a key benefit. Using the OpenAI-compatible endpoint allows us to leverage the robust OpenAI Agents SDK without vendor lock-in.
- **Alternatives Considered**:
  - **Gemini 1.5 Pro**: Rejected because its additional capabilities are not required for this task, and it has slightly higher latency than Flash.
  - **Other OpenAI Models (e.g., GPT-4o)**: Rejected in favor of using the Gemini ecosystem as specified in the project constraints.

---

### 5. Decision: Scope Reduction for Speed

- **Decision**: Several features from the original spec will be deferred to post-MVP.
- **Rationale**: To meet the 2-day timeline, scope must be aggressively controlled. The following were cut from the MVP:
  - **Querying Selected Text**: This adds significant complexity to the frontend and backend logic. The core value is in full-book retrieval, which is prioritized.
  - **Neon Serverless Postgres**: This is entirely unnecessary for the MVP. Qdrant's ability to store metadata alongside vectors is sufficient for linking back to source documents.
  - **Advanced Citations**: The MVP will provide basic links (URLs or section titles) to source documents. More granular, snippet-level citations are a post-MVP enhancement.
- **Alternatives Considered**:
  - **Building all features with lower quality**: Rejected in favor of delivering a smaller set of high-quality, functional features. A working, fast chatbot is a better MVP than a slow, buggy one with more features.
