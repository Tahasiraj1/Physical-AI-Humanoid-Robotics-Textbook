# Implementation Plan: Chatbot Personalization with User Context

**Branch**: `017-chatbot-user-context` | **Date**: 2025-01-27 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/017-chatbot-user-context/spec.md`

## Summary

Enhance the chatbot frontend to send user personalization data (progress, bookmarks, notes) to the backend on the first message of each new session, and update the backend to accept and use this context for personalized responses. The frontend will fetch user context from existing personalization APIs, cache it per session, and include it only on the first message of a new session. The backend will accept the optional user_context field and incorporate it into chatbot responses for personalization.

**Technical Approach**: Frontend modifications to ChatWidget component to fetch and send user context on first message of new session, backend FastAPI endpoint updates to accept optional user_context field in ChatRequest model, and integration with existing personalization services (progressService, bookmarkService, noteService).

## Technical Context

**Language/Version**: 
- TypeScript 5.6.2 (frontend - React, Docusaurus)
- Python 3.11+ (backend - FastAPI, Pydantic)

**Primary Dependencies**: 
- Frontend:
  - React 19.0.0 (existing)
  - TypeScript 5.6.2 (existing)
  - Docusaurus 3.9.2 (existing)
  - Existing personalization services (progressService, bookmarkService, noteService)
  - Existing AuthProvider for user authentication
- Backend:
  - FastAPI (existing)
  - Pydantic (existing, for request/response models)
  - OpenAI Agents SDK (existing, for agent processing)

**Storage**: 
- Browser sessionStorage (frontend - for tracking whether context was sent per session)
- In-memory session storage (backend - existing, no changes needed)

**Testing**: 
- Frontend: Jest/React Testing Library (to be configured)
- Backend: pytest (existing)

**Target Platform**: 
- Web browsers (frontend - modern browsers supporting ES6+, sessionStorage)
- Linux server (backend - FastAPI on Hugging Face Spaces or local)

**Project Type**: Web application (frontend component + backend API)

**Performance Goals**: 
- User context fetching adds <500ms to chatbot response time (SC-002)
- Context sent only once per session (SC-005)
- Backend processes user_context without errors for 99% of requests (SC-006)

**Constraints**: 
- User context must be sent only on first message of new session (FR-012, FR-013)
- Must work for unauthenticated users without errors (FR-002)
- Must gracefully degrade if context fetching fails (FR-008)
- Note content limited to 200 characters to prevent large payloads (FR-006)
- Frontend must track per session whether context was sent (FR-007)

**Scale/Scope**: 
- Frontend: Updates to ChatWidget component and related hooks/services
- Backend: Updates to ChatRequest model and chat endpoint
- No new APIs or services - uses existing personalization APIs

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Documentation-First Architecture ✅
- Feature enhances chatbot functionality without modifying documentation structure
- No impact on book content or module organization
- Chatbot responses with personalization reference existing book content

### II. Modular Content Organization ✅
- Feature is a standalone enhancement to existing chatbot
- No impact on module structure or content organization
- Uses existing personalization data tied to book modules/sections

### III. Vector Database Integration (NON-NEGOTIABLE) ✅
- No changes to Qdrant or embedding pipeline
- Feature uses existing RAG system, only adds user context to requests
- Citations and retrieval remain unchanged

### IV. AI Agent Architecture ✅
- Uses existing OpenAI Agents SDK
- No changes to agent behavior logic - only passes additional context
- Agent responses remain traceable to source content

### V. Deployment Standards ✅
- No changes to deployment configurations
- Frontend changes deploy with Docusaurus to GitHub Pages
- Backend changes deploy to Hugging Face Spaces via Docker

### VI. API-First Backend Design ✅
- Extends existing FastAPI endpoint with optional field
- Maintains backward compatibility (user_context is optional)
- API schema documented via Pydantic models

**Constitution Compliance**: ✅ All gates pass. No violations.

## Project Structure

### Documentation (this feature)

```text
specs/017-chatbot-user-context/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── api-contracts.md # API contract for user_context field
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Frontend (Docusaurus)
src/components/ChatWidget/
├── index.tsx              # Main ChatWidget component (update to fetch/send context)
├── hooks/
│   ├── useChatSession.ts  # Session management (update to track context sent)
│   └── useChatAPI.ts      # API communication (update to include context)
├── services/
│   └── chatService.ts     # API service (update to fetch and send user context)
└── types.ts               # TypeScript types (update ChatRequest interface)

src/components/Personalization/
├── services/
│   ├── progressService.ts  # Existing - used to fetch progress
│   ├── bookmarkService.ts # Existing - used to fetch bookmarks
│   └── noteService.ts     # Existing - used to fetch notes

src/components/Auth/
└── AuthProvider.tsx        # Existing - provides user ID

# Backend (FastAPI)
Chatbot/src/chatbot/api/
├── routes.py              # Chat endpoint (update ChatRequest model, process user_context)
└── dependencies.py        # Existing - no changes

Chatbot/src/chatbot/
└── (no new files needed)
```

**Structure Decision**: Web application structure with frontend (Docusaurus/React) and backend (FastAPI) components. Frontend modifications are limited to ChatWidget component and related hooks/services. Backend modifications are limited to API route handler and request model. No new services or major architectural changes required.

## Complexity Tracking

> **No violations detected - all constitution gates pass**
