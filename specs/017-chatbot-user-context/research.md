# Research: Chatbot Personalization with User Context

**Date**: 2025-01-27  
**Feature**: 017-chatbot-user-context  
**Sources**: Existing codebase, React/TypeScript best practices, FastAPI/Pydantic patterns

## Research Questions

### 1. How to track whether user context was sent for a session in the frontend?

**Decision**: Use a Map or object keyed by session_id to track whether context has been sent for each session. Store this in component state or a custom hook, and check before sending context.

**Rationale**: 
- Simple in-memory tracking is sufficient since sessions are browser-scoped
- No need for persistent storage (sessionStorage) since tracking is per-session lifecycle
- Map structure provides O(1) lookup performance
- State resets when component unmounts, which is acceptable since new sessions will be created

**Alternatives considered**:
- sessionStorage: Rejected - adds complexity, not needed since tracking is ephemeral
- Backend tracking: Rejected - adds unnecessary state management, frontend can handle this
- Cookie-based: Rejected - overkill for simple boolean flag per session

**Implementation Pattern**:
```typescript
// In useChatSession hook or ChatWidget component
const [contextSentForSessions, setContextSentForSessions] = useState<Set<string>>(new Set());

function hasContextBeenSent(sessionId: string): boolean {
  return contextSentForSessions.has(sessionId);
}

function markContextAsSent(sessionId: string): void {
  setContextSentForSessions(prev => new Set(prev).add(sessionId));
}
```

### 2. How to fetch user context data in parallel and handle partial failures?

**Decision**: Use `Promise.allSettled()` to fetch progress, bookmarks, and notes in parallel, then process results to include only successful fetches. This ensures graceful degradation if one API fails.

**Rationale**:
- `Promise.allSettled()` doesn't fail if any promise rejects, allowing partial success
- Parallel fetching minimizes latency (SC-002: <500ms)
- Graceful degradation aligns with FR-008 and FR-010
- User gets personalized responses with available data even if some APIs fail

**Alternatives considered**:
- Sequential fetching: Rejected - increases latency unnecessarily
- Promise.all(): Rejected - fails completely if any API fails, violating graceful degradation
- Retry logic: Deferred - can be added later if needed, not required for MVP

**Implementation Pattern**:
```typescript
const [progressResult, bookmarksResult, notesResult] = await Promise.allSettled([
  progressService.getUserProgress(),
  bookmarkService.getBookmarks(),
  noteService.getNotes(),
]);

const userContext: UserContext = {};
if (progressResult.status === 'fulfilled') {
  userContext.progress = processProgressData(progressResult.value);
}
if (bookmarksResult.status === 'fulfilled') {
  userContext.bookmarks = processBookmarksData(bookmarksResult.value);
}
if (notesResult.status === 'fulfilled') {
  userContext.notes = processNotesData(notesResult.value);
}
```

### 3. How to structure user_context in the backend ChatRequest model?

**Decision**: Add optional `user_context` field to ChatRequest Pydantic model as a nested dict structure matching the frontend UserContext type. Use Pydantic's Optional and Field for validation.

**Rationale**:
- Optional field maintains backward compatibility (FR-016)
- Pydantic provides automatic validation and serialization
- Nested structure matches frontend type for consistency
- Can be extended in future without breaking changes

**Alternatives considered**:
- Separate endpoint: Rejected - adds complexity, violates API-first principle
- Query parameter: Rejected - too large for query string, better in request body
- Header-based: Rejected - context is request-specific, not header material

**Implementation Pattern**:
```python
class UserContext(BaseModel):
    progress: Optional[dict] = None
    bookmarks: Optional[list[dict]] = None
    notes: Optional[list[dict]] = None

class ChatRequest(BaseModel):
    message: str
    session_id: str
    thread_id: Optional[str] = None
    user_id: Optional[str] = None
    user_context: Optional[UserContext] = None  # NEW FIELD
```

### 4. How to incorporate user context into chatbot responses?

**Decision**: Pass user_context to the agent as part of the message context or system prompt enhancement. The agent can use this context to personalize responses without modifying agent logic.

**Rationale**:
- Minimal changes to existing agent architecture
- Context is informational, agent can use it naturally in responses
- No need to modify agent tools or behavior
- Aligns with out-of-scope: "Backend AI model changes - only passing context"

**Alternatives considered**:
- Modify agent tools: Rejected - out of scope, adds complexity
- Separate context endpoint: Rejected - adds unnecessary API calls
- Store in session: Rejected - out of scope, backend doesn't persist context

**Implementation Pattern**:
```python
# In chat_endpoint, before calling agent
enhanced_message = request.message
if request.user_context:
    context_prompt = build_context_prompt(request.user_context)
    enhanced_message = f"{context_prompt}\n\nUser question: {request.message}"

result = await Runner.run(agent, enhanced_message)
```

### 5. How to limit note content to prevent large payloads?

**Decision**: Truncate note content to 200 characters when building user_context in the frontend, before sending to backend. Use substring with ellipsis if needed.

**Rationale**:
- Prevents large payloads that could slow down requests
- 200 characters is sufficient for context without losing essential information
- Truncation in frontend is simpler than backend validation
- Aligns with FR-006 and assumptions

**Implementation Pattern**:
```typescript
const truncatedNotes = notes.map(note => ({
  moduleId: note.moduleId,
  sectionId: note.sectionId,
  content: note.content.substring(0, 200), // Limit to 200 chars
}));
```

## Summary

All research questions resolved. Implementation approach:
1. Frontend tracks context-sent status per session using Set in component state
2. Parallel fetching with Promise.allSettled() for graceful degradation
3. Optional user_context field in ChatRequest Pydantic model
4. Context passed to agent via enhanced message prompt
5. Note content truncated to 200 characters in frontend

No additional research needed - all technical decisions are clear and align with existing patterns.

