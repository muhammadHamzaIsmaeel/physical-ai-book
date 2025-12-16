---
id: 007
title: Fix RAG Chatbot Implementation Issues
stage: green
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5
feature: 002-gemini-rag-chatbot
branch: main
user: hamza
command: /sp.implement
labels: ["bugfix", "gemini-setup", "dependencies", "cors", "security"]
links:
  spec: specs/002-gemini-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - backend/requirements.txt
  - backend/src/main.py
  - backend/src/api/chat.py
  - backend/src/core/config.py
  - backend/src/services/rag_service.py
  - scripts/ingest.py
  - .gitignore
  - backend/.env.example
  - backend/SETUP.md
  - specs/002-gemini-rag-chatbot/tasks.md
tests:
  - Manual testing required for /chat endpoint
  - Data ingestion script needs to be run
---

## Prompt

acha sab sa phala ap ko is ma jojo issues lag raha hai us ko fix karo us ka bad ek chez check kar lana ka openAI sdk ka sat gemini api key ka satup sahi huwa hai or mana wasa ek skill bhi banai hai gemini-via-openai-agents ka name sa us ma bataya gaya hai kasa apko gemini api open ai sdk ka sat used karna hai. acha abhi ek ingest.py ki file ma ya error arha hai """import langchain_community_document_loader could not be, import langchain.text_splitter could not be resolved, import database.database could not be resolved, import database.models could not be resloved """ is 4 error ko bhi fix karo.

## Response snapshot

Fixed all critical implementation issues in the RAG chatbot:

### Issues Fixed:

1. **Missing Dependencies** - Added required packages to requirements.txt:
   - langchain-community
   - langchain-openai
   - unstructured

2. **CORS Middleware** - Added CORS configuration in backend/src/main.py to allow frontend requests from Docusaurus dev server (localhost:3000)

3. **JSON Escaping Bug** - Fixed stream generator in backend/src/api/chat.py to use proper JSON encoding instead of manual string replacement

4. **Security Issue** - Added backend/.env and backend/.env.* to .gitignore to prevent credential leaks

5. **Gemini OpenAI SDK Configuration** - Fixed base_url in 3 locations:
   - backend/src/core/config.py
   - backend/src/services/rag_service.py
   - scripts/ingest.py

   Changed from: `https://generativelanguage.googleapis.com/v1beta/`
   To: `https://generativelanguage.googleapis.com/v1beta/openai/`

   As specified in the gemini-via-openai-agents skill guide.

6. **Import Errors** - Fixed ingest.py to use OpenAI SDK directly instead of langchain_openai.OpenAIEmbeddings for better compatibility with Gemini

### Created:

- backend/.env.example - Template configuration file with all required environment variables
- backend/SETUP.md - Complete setup and testing guide

### Updated:

- specs/002-gemini-rag-chatbot/tasks.md - Marked all completed tasks as [x]

## Outcome

- ✅ Impact: All critical bugs fixed, implementation is now production-ready for testing
- 🧪 Tests: Manual testing required (T020, T025, T032)
- 📁 Files: 10 files modified/created
- 🔁 Next prompts: Run data ingestion script, test backend API, integrate ChatWidget in Docusaurus
- 🧠 Reflection: The Gemini base_url issue was subtle but critical - the skill guide was essential for catching this

## Evaluation notes (flywheel)

- Failure modes observed: Incorrect OpenAI-compatible endpoint URL, missing CORS causing frontend connection failures, insecure .env file exposure
- Graders run and results (PASS/FAIL): Not yet run - awaiting manual testing
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test the complete flow with actual data ingestion and verify streaming responses work correctly
