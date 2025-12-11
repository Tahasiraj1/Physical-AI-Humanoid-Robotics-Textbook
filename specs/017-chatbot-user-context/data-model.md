# Data Model: Chatbot Personalization with User Context

**Feature**: 017-chatbot-user-context  
**Date**: 2025-01-27

## Overview

This data model defines the entities and data structures for sending user personalization context to the chatbot backend. The model extends the existing ChatRequest structure with an optional user_context field containing progress, bookmarks, and notes data.

## Entities

### User Context

**Purpose**: Represents aggregated personalization data for a user, sent to the backend on the first message of a new session.

**Attributes**:
- `progress` (object, optional): User's learning progress information
  - `completedModules` (array of strings, optional): List of module IDs the user has completed
  - `completedSections` (array of objects, optional): List of completed sections with moduleId and sectionId
  - `progressSummary` (object, optional): Progress summary per module with progress percentages
- `bookmarks` (array of objects, optional): User's bookmarked sections
  - Each bookmark: `{ moduleId: string, sectionId: string, title?: string }`
- `notes` (array of objects, optional): User's notes on sections
  - Each note: `{ moduleId: string, sectionId: string, content: string }` (content limited to 200 chars)

**Relationships**:
- Sent as part of ChatRequest (optional field)
- Derived from existing personalization APIs (progress, bookmarks, notes)
- Associated with user_id (implicit, from authenticated user)

**Lifecycle**:
- Created: When frontend fetches user data on first message of new session
- Sent: Included in ChatRequest to backend (only once per session)
- Used: Backend incorporates into chatbot response context
- Destroyed: Not persisted, used for single request only

**Validation Rules**:
- All fields are optional (user may have no progress, bookmarks, or notes)
- Note content must be truncated to 200 characters maximum
- Empty arrays are valid (user has no bookmarks/notes)
- Empty objects are valid (user has no progress data)

**Storage**:
- Frontend: Temporary in-memory during request construction
- Backend: Not persisted, used only for current request processing

---

### Context Sent Tracking (Frontend)

**Purpose**: Tracks whether user context has been sent for each session to prevent resending.

**Attributes**:
- `sessionId` (string): Session identifier
- `contextSent` (boolean): Whether context was sent for this session

**Relationships**:
- One-to-one with chat session
- Stored in frontend component state

**Lifecycle**:
- Created: When new session is detected
- Updated: When context is sent (marked as true)
- Destroyed: When component unmounts or session ends

**Validation Rules**:
- Each sessionId can only have context sent once
- State resets on component unmount (acceptable, new sessions will be created)

**Storage**:
- Frontend: In-memory Set<string> in React component state
- Not persisted to sessionStorage (ephemeral tracking)

---

### ChatRequest (Extended)

**Purpose**: Extended request model for chatbot API that includes optional user context.

**Attributes** (existing + new):
- `message` (string, required): User message text
- `session_id` (string, required): Session identifier
- `thread_id` (string, optional): Optional thread ID
- `user_id` (string, optional): Optional user ID for authenticated users
- `user_context` (UserContext, optional): **NEW** - User personalization context

**Relationships**:
- Contains optional UserContext
- Sent from frontend to backend
- Processed by FastAPI chat endpoint

**Validation Rules**:
- user_context is optional (backward compatible)
- If user_context is provided, it must match UserContext structure
- user_context should only be sent on first message of new session (frontend responsibility)

**Storage**:
- Transient: Sent in HTTP request body
- Not persisted by backend

---

## Data Flow

1. **User sends first message in new session**:
   - Frontend detects new session (no existing sessionId or new sessionId)
   - Frontend checks if context was sent for this session (no)
   - Frontend fetches user data in parallel (progress, bookmarks, notes)
   - Frontend constructs UserContext object
   - Frontend includes user_context in ChatRequest
   - Frontend marks context as sent for this session

2. **User sends subsequent messages in same session**:
   - Frontend checks if context was sent for this session (yes)
   - Frontend sends ChatRequest without user_context field

3. **Backend receives request**:
   - Backend validates ChatRequest (user_context is optional)
   - If user_context provided, backend incorporates into agent prompt
   - Backend processes request and returns personalized response

4. **User starts new session**:
   - Frontend detects new sessionId
   - Process repeats from step 1

---

## Type Definitions

### TypeScript (Frontend)

```typescript
interface UserContext {
  progress?: {
    completedModules?: string[];
    completedSections?: Array<{ moduleId: string; sectionId: string }>;
    progressSummary?: Record<string, { progressPercentage: number }>;
  };
  bookmarks?: Array<{
    moduleId: string;
    sectionId: string;
    title?: string;
  }>;
  notes?: Array<{
    moduleId: string;
    sectionId: string;
    content: string; // Max 200 characters
  }>;
}

interface ChatRequest {
  message: string;
  session_id: string;
  thread_id?: string;
  user_id?: string;
  user_context?: UserContext; // NEW FIELD
}
```

### Python (Backend)

```python
from pydantic import BaseModel, Field
from typing import Optional, List, Dict

class ProgressContext(BaseModel):
    completed_modules: Optional[List[str]] = None
    completed_sections: Optional[List[Dict[str, str]]] = None
    progress_summary: Optional[Dict[str, Dict[str, float]]] = None

class BookmarkContext(BaseModel):
    module_id: str
    section_id: str
    title: Optional[str] = None

class NoteContext(BaseModel):
    module_id: str
    section_id: str
    content: str  # Max 200 characters

class UserContext(BaseModel):
    progress: Optional[ProgressContext] = None
    bookmarks: Optional[List[BookmarkContext]] = None
    notes: Optional[List[NoteContext]] = None

class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=2000)
    session_id: str
    thread_id: Optional[str] = None
    user_id: Optional[str] = None
    user_context: Optional[UserContext] = None  # NEW FIELD
```

---

## Constraints

1. **Note Content Length**: Maximum 200 characters per note (enforced in frontend)
2. **Session-Based Sending**: user_context sent only once per session (enforced in frontend)
3. **Optional Field**: user_context is optional for backward compatibility (enforced in backend)
4. **Graceful Degradation**: Missing or partial user_context does not break chatbot (enforced in backend)

---

## Migration Notes

- **No database migration required**: All data is transient, not persisted
- **Backward compatible**: Existing requests without user_context continue to work
- **Frontend changes**: Add user_context fetching and session tracking
- **Backend changes**: Add optional user_context field to ChatRequest model

