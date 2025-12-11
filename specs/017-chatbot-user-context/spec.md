# Feature Specification: Chatbot Personalization with User Context

**Feature Branch**: `017-chatbot-user-context`  
**Created**: 2025-01-27  
**Status**: Draft  
**Input**: User description: "# Feature: Chatbot Personalization with User Context"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Personalized Chatbot Responses (Priority: P1)

An authenticated user who has completed modules, bookmarked sections, and created notes wants the chatbot to reference their learning progress and personal content when answering questions. The chatbot should be aware of what the user has learned, what they've bookmarked, and any notes they've taken, so it can provide more relevant and personalized responses.

**Why this priority**: This is the core value proposition - enabling personalized responses based on user's learning journey. Without this, the chatbot cannot leverage the user's personalization data that already exists in the system.

**Independent Test**: As an authenticated user who has completed 3 modules, bookmarked 5 sections, and created 2 notes, send a message to the chatbot asking "What should I review next?" The chatbot response should reference completed modules, suggest bookmarked sections, or mention notes. Verify that the chatbot's response demonstrates awareness of the user's progress and personal content.

**Acceptance Scenarios**:

1. **Given** an authenticated user has completed modules A and B, **When** they ask the chatbot "What topics should I focus on next?", **Then** the chatbot response acknowledges their completed modules and suggests relevant next steps
2. **Given** an authenticated user has bookmarked 3 sections, **When** they ask "Can you explain the concepts I bookmarked?", **Then** the chatbot response references the bookmarked sections and provides explanations
3. **Given** an authenticated user has created notes on a specific section, **When** they ask a question related to that section, **Then** the chatbot response can reference or build upon the user's notes
4. **Given** an authenticated user sends their first message in a new session, **When** the chatbot processes the request, **Then** user context (progress, bookmarks, notes) is automatically included in the request to the backend
5. **Given** an authenticated user sends subsequent messages in the same session, **When** messages are sent, **Then** user context is NOT included in the request (only sent on first message)

---

### User Story 2 - Graceful Degradation for Unauthenticated Users (Priority: P2)

An unauthenticated user wants to use the chatbot without any personalization features. The chatbot should work normally without user context, and no errors should occur when user context cannot be fetched.

**Why this priority**: Ensures the chatbot remains functional for all users, not just authenticated ones. This maintains backward compatibility and prevents feature regression.

**Independent Test**: As an unauthenticated user, open the chatbot and send a message. Verify the chatbot responds normally without any errors. Verify no user context is sent in the request. Verify the chatbot works identically to how it worked before this feature.

**Acceptance Scenarios**:

1. **Given** an unauthenticated user opens the chatbot, **When** they send a message, **Then** the chatbot responds normally without user context
2. **Given** an unauthenticated user sends a message, **When** the request is made to the backend, **Then** no user_context field is included in the request payload
3. **Given** an unauthenticated user uses the chatbot, **When** they interact with it, **Then** no errors occur related to missing user context or authentication

---

### User Story 3 - Context Fetching with Error Resilience (Priority: P2)

An authenticated user wants the chatbot to work reliably even if fetching their personalization data fails temporarily. The chatbot should continue functioning without personalization rather than failing completely.

**Why this priority**: Ensures system resilience and prevents a single point of failure from breaking the entire chatbot experience. User experience should degrade gracefully, not fail catastrophically.

**Independent Test**: As an authenticated user, simulate a failure in fetching progress data (e.g., network error or API timeout). Send a message to the chatbot. Verify the chatbot still responds, but without personalized context. Verify no error is shown to the user about context fetching failures.

**Acceptance Scenarios**:

1. **Given** an authenticated user's progress API call fails, **When** they send a chatbot message, **Then** the chatbot responds without progress context but still functions normally
2. **Given** an authenticated user's bookmarks API call fails, **When** they send a chatbot message, **Then** the chatbot responds without bookmark context but still functions normally
3. **Given** an authenticated user's notes API call fails, **When** they send a chatbot message, **Then** the chatbot responds without notes context but still functions normally
4. **Given** all personalization API calls fail for an authenticated user, **When** they send a chatbot message, **Then** the chatbot responds without any user context but still functions normally
5. **Given** an authenticated user's context fetch fails, **When** they send a message, **Then** no error message is displayed to the user about the context fetch failure

---

### Edge Cases

- What happens when a user has completed 0 modules but has bookmarks and notes? (Chatbot should still reference bookmarks and notes)
- What happens when a user has 100+ bookmarks? (System should limit or summarize bookmarks to avoid large payloads)
- What happens when a user has very long notes (exceeding reasonable payload size)? (System should truncate note content)
- How does system handle context fetching when user switches accounts mid-session? (New session should be created when user changes, context sent with first message of new session)
- What happens when a new session is created (user closes and reopens chatbot, or sessionStorage is cleared)? (User context should be sent with first message of new session)
- How does system handle partial context fetch success (e.g., progress succeeds but bookmarks fail)? (System should include available context, exclude failed parts)
- What happens when user context is empty (no progress, no bookmarks, no notes)? (Chatbot should work normally without context, no user_context field sent)
- How does system track whether context was already sent for a session? (Frontend must track per session_id whether context was sent)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST include user personalization data (progress, bookmarks, notes) in chatbot requests when user is authenticated AND it is the first message of a new session
- **FR-002**: System MUST exclude user context from chatbot requests when user is not authenticated
- **FR-003**: System MUST fetch user progress data (completed modules, completed sections, progress summary) for authenticated users
- **FR-004**: System MUST fetch user bookmarks (module ID, section ID, title) for authenticated users
- **FR-005**: System MUST fetch user notes (module ID, section ID, content) for authenticated users
- **FR-006**: System MUST limit note content length to prevent excessive payload sizes
- **FR-007**: System MUST track whether user context has been sent for the current session to avoid resending on subsequent messages
- **FR-008**: System MUST continue chatbot functionality even if user context fetching fails (graceful degradation)
- **FR-009**: System MUST fetch user context in parallel for progress, bookmarks, and notes to minimize latency
- **FR-010**: System MUST handle partial context fetch failures (include available data, exclude failed parts)
- **FR-011**: System MUST invalidate context tracking when user changes (different user ID detected) to ensure new user's context is sent on first message
- **FR-012**: System MUST send user context only on the first message of a new session (when session_id is newly created or doesn't exist)
- **FR-013**: Frontend MUST send user_context field in chatbot request only on the first message of a new session (implementation of FR-012)
- **FR-014**: Backend MUST accept and process user_context field in chatbot requests (optional field)
- **FR-015**: Backend MUST use user context to personalize chatbot responses when user_context is provided
- **FR-016**: Backend MUST function normally when user_context is not provided or is empty

### Key Entities *(include if feature involves data)*

- **User Context**: Represents aggregated personalization data for a user, containing progress information, bookmarks, and notes. Used to enhance chatbot responses with personalized references.

- **Progress Context**: Contains information about user's learning progress including completed modules, completed sections, and progress summary percentages per module.

- **Bookmark Context**: Contains list of bookmarked sections with module ID, section ID, and optional title for quick reference in chatbot responses.

- **Note Context**: Contains list of user notes with module ID, section ID, and truncated content (limited length) to provide context for personalized responses.

- **Context Sent Tracking**: In-memory Set tracking which session IDs have already sent user context, stored in frontend component state. Prevents resending context on subsequent messages in the same session.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Authenticated users receive chatbot responses that reference their personalization data (progress, bookmarks, or notes) in at least 80% of conversations where such data exists
- **SC-002**: User context fetching adds less than 500ms to chatbot response time for authenticated users (measured from message send to response received)
- **SC-003**: Chatbot continues to function normally for 100% of unauthenticated users (no regression in functionality)
- **SC-004**: Chatbot continues to function normally for authenticated users even when context fetching fails (100% uptime for core chatbot functionality)
- **SC-005**: User context is sent only once per session (on first message), eliminating redundant context payloads on subsequent messages
- **SC-006**: Backend successfully processes user_context field in chatbot requests without errors for 99% of authenticated requests
- **SC-007**: Users report improved relevance of chatbot responses when personalization data is available (qualitative feedback indicates positive impact)

## Assumptions

- User authentication system is already implemented and provides user ID for authenticated users
- Personalization APIs (progress, bookmarks, notes) are already implemented and functional
- Backend chatbot service can be updated to accept and process user_context field
- Note content can be reasonably truncated to 200 characters without losing essential context
- Context cache time-to-live of 5 minutes provides good balance between freshness and performance
- Parallel fetching of progress, bookmarks, and notes APIs is supported and won't cause rate limiting issues
- Backend has capacity to process user context and incorporate it into chatbot responses without significant performance degradation

## Dependencies

- Existing authentication system (AuthProvider) that provides user ID
- Existing personalization services (progressService, bookmarkService, noteService) that fetch user data
- Existing chatbot backend API that can be extended to accept optional user_context field in ChatRequest model
- Existing chatbot frontend (ChatWidget) that can be updated to include user context in requests on first message of new session

## Clarifications

### Session 2025-01-27

- Q: When should user_context be sent to the backend? → A: Only on the first message of a new session (when session_id is newly created or doesn't exist)
- Q: What triggers a "session refresh" for sending user_context? → A: Only when a new session is created (user opens chatbot with no existing session_id)

## Out of Scope

- Modifying personalization APIs (progress, bookmarks, notes) - these are assumed to work as-is, frontend only calls existing APIs
- Changing authentication system - user authentication is already implemented
- Implementing new personalization features - only using existing progress, bookmarks, and notes
- Backend AI model changes - only passing context to existing backend, not modifying AI behavior logic
- Backend session storage of user_context - backend receives and uses context for that request only, does not need to persist it
- User interface changes for displaying personalized responses - only backend receives context and personalizes responses
- Real-time context updates during active chat sessions - context is sent once per session on first message only
