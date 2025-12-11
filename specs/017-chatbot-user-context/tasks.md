# Tasks: Chatbot Personalization with User Context

**Input**: Design documents from `/specs/017-chatbot-user-context/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Tests are OPTIONAL - not explicitly requested in specification, so test tasks are not included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `src/components/ChatWidget/` (Docusaurus React components)
- **Backend**: `Chatbot/src/chatbot/api/` (FastAPI routes)

---

## Phase 1: Setup (Type Definitions)

**Purpose**: Define TypeScript and Python type definitions for user context

- [x] T001 [P] Update ChatRequest interface with user_context field in `src/components/ChatWidget/types.ts`
- [x] T002 [P] Create UserContext TypeScript interface in `src/components/ChatWidget/types.ts`
- [x] T003 [P] Update ChatRequest Pydantic model with user_context field in `Chatbot/src/chatbot/api/routes.py`
- [x] T004 [P] Create UserContext Pydantic models (UserContext, ProgressContext, BookmarkContext, NoteContext) in `Chatbot/src/chatbot/api/routes.py`

**Checkpoint**: Type definitions complete - ready for implementation

---

## Phase 2: User Story 1 - Personalized Chatbot Responses (Priority: P1) 🎯 MVP

**Goal**: Enable authenticated users to receive personalized chatbot responses that reference their learning progress, bookmarks, and notes on the first message of each new session.

**Independent Test**: As an authenticated user who has completed 3 modules, bookmarked 5 sections, and created 2 notes, send a message to the chatbot asking "What should I review next?" The chatbot response should reference completed modules, suggest bookmarked sections, or mention notes. Verify that the chatbot's response demonstrates awareness of the user's progress and personal content.

### Implementation for User Story 1

- [x] T005 [P] [US1] Create fetchUserContext function in `src/components/ChatWidget/services/chatService.ts` to fetch progress, bookmarks, and notes in parallel using Promise.allSettled
- [x] T006 [US1] Update sendMessage function in `src/components/ChatWidget/services/chatService.ts` to accept includeUserContext parameter and fetch/include user context when requested
- [x] T007 [P] [US1] Add hasContextBeenSent and markContextAsSent methods to useChatSession hook in `src/components/ChatWidget/hooks/useChatSession.ts`
- [x] T008 [US1] Update ChatWidget component in `src/components/ChatWidget/index.tsx` to import useAuthContext and get user ID
- [x] T009 [US1] Update handleSendMessage in ChatWidget component in `src/components/ChatWidget/index.tsx` to check if context was sent, fetch context on first message, and mark context as sent
- [x] T010 [US1] Update useChatAPI hook in `src/components/ChatWidget/hooks/useChatAPI.ts` to pass includeUserContext parameter to sendMessage
- [x] T011 [P] [US1] Add UserContext Pydantic models (UserContext, ProgressContext, BookmarkContext, NoteContext) to ChatRequest in `Chatbot/src/chatbot/api/routes.py`
- [x] T012 [US1] Create build_context_prompt function in `Chatbot/src/chatbot/api/routes.py` to convert user_context into prompt text
- [x] T013 [US1] Update chat_endpoint function in `Chatbot/src/chatbot/api/routes.py` to incorporate user_context into agent prompt when provided

**Checkpoint**: At this point, User Story 1 should be fully functional - authenticated users receive personalized responses on first message of new session

---

## Phase 3: User Story 2 - Graceful Degradation for Unauthenticated Users (Priority: P2)

**Goal**: Ensure unauthenticated users can use the chatbot normally without user context, with no errors when context cannot be fetched.

**Independent Test**: As an unauthenticated user, open the chatbot and send a message. Verify the chatbot responds normally without any errors. Verify no user context is sent in the request. Verify the chatbot works identically to how it worked before this feature.

### Implementation for User Story 2

- [x] T014 [US2] Update handleSendMessage in ChatWidget component in `src/components/ChatWidget/index.tsx` to skip context fetching when user is not authenticated
- [x] T015 [US2] Verify sendMessage function in `src/components/ChatWidget/services/chatService.ts` handles undefined userId gracefully (no user_context sent)
- [x] T016 [US2] Verify chat_endpoint function in `Chatbot/src/chatbot/api/routes.py` processes requests without user_context field normally (backward compatibility)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - authenticated users get personalized responses, unauthenticated users work normally

---

## Phase 4: User Story 3 - Context Fetching with Error Resilience (Priority: P2)

**Goal**: Ensure the chatbot continues functioning even if fetching user personalization data fails temporarily, with graceful degradation.

**Independent Test**: As an authenticated user, simulate a failure in fetching progress data (e.g., network error or API timeout). Send a message to the chatbot. Verify the chatbot still responds, but without personalized context. Verify no error is shown to the user about context fetching failures.

### Implementation for User Story 3

- [x] T017 [US3] Update fetchUserContext function in `src/components/ChatWidget/services/chatService.ts` to handle Promise.allSettled results and include only successful fetches (partial context support)
- [x] T018 [US3] Add error handling in fetchUserContext to return undefined on any error (graceful degradation) in `src/components/ChatWidget/services/chatService.ts`
- [x] T019 [US3] Update handleSendMessage in ChatWidget component in `src/components/ChatWidget/index.tsx` to handle undefined userContext gracefully (send request without context)
- [x] T020 [US3] Verify chat_endpoint function in `Chatbot/src/chatbot/api/routes.py` handles empty or partial user_context gracefully (processes normally)

**Checkpoint**: At this point, all user stories should work independently - system gracefully degrades when context fetching fails

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements and validation across all user stories

- [x] T021 [P] Add note content truncation to 200 characters in fetchUserContext function in `src/components/ChatWidget/services/chatService.ts`
- [x] T022 [P] Add logging for user context fetching in `src/components/ChatWidget/services/chatService.ts` (console.warn on failures)
- [x] T023 [P] Add logging for user context processing in `Chatbot/src/chatbot/api/routes.py` (log when context is provided/used)
- [x] T024 [US3] Add user change detection in ChatWidget component in `src/components/ChatWidget/index.tsx` to clear context-sent tracking when user ID changes (use useEffect to watch user.id from AuthProvider)
- [x] T025 Verify session tracking works correctly when user switches accounts mid-session in `src/components/ChatWidget/index.tsx`
- [x] T026 Verify context is sent on new session creation (sessionStorage cleared) in `src/components/ChatWidget/index.tsx`
- [x] T027 Run quickstart.md validation - test end-to-end flow with authenticated user
- [x] T028 Run quickstart.md validation - test end-to-end flow with unauthenticated user
- [x] T029 Run quickstart.md validation - test error resilience scenarios

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **User Story 1 (Phase 2)**: Depends on Setup completion - Core MVP feature
- **User Story 2 (Phase 3)**: Depends on Setup completion - Can run in parallel with US1 after Setup
- **User Story 3 (Phase 4)**: Depends on Setup completion - Can run in parallel with US1/US2 after Setup
- **Polish (Phase 5)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Setup (Phase 1) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Setup (Phase 1) - Independent, tests unauthenticated flow
- **User Story 3 (P2)**: Can start after Setup (Phase 1) - Independent, tests error handling

### Within Each User Story

- Type definitions before implementation
- Frontend changes before backend changes (or in parallel)
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T001-T004)
- Frontend and backend tasks in US1 can run in parallel (T005-T010 frontend, T011-T013 backend)
- User Stories 2 and 3 can be worked on in parallel after Setup
- Polish tasks marked [P] can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch frontend tasks in parallel:
Task: "Create fetchUserContext function in src/components/ChatWidget/services/chatService.ts"
Task: "Add hasContextBeenSent and markContextAsSent methods to useChatSession hook"
Task: "Add UserContext Pydantic models to ChatRequest in Chatbot/src/chatbot/api/routes.py"

# Then launch dependent tasks:
Task: "Update sendMessage function in src/components/ChatWidget/services/chatService.ts"
Task: "Update ChatWidget component in src/components/ChatWidget/index.tsx"
Task: "Update chat_endpoint function in Chatbot/src/chatbot/api/routes.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (type definitions)
2. Complete Phase 2: User Story 1 (personalized responses)
3. **STOP and VALIDATE**: Test User Story 1 independently
   - Authenticated user sends first message → context included
   - Authenticated user sends second message → context NOT included
   - Response references user's progress/bookmarks/notes
4. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup → Type definitions ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo (unauthenticated support)
4. Add User Story 3 → Test independently → Deploy/Demo (error resilience)
5. Add Polish → Final validation → Deploy

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup together (type definitions)
2. Once Setup is done:
   - Developer A: User Story 1 (frontend + backend)
   - Developer B: User Story 2 (frontend validation)
   - Developer C: User Story 3 (error handling)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- Frontend and backend can be developed in parallel for US1
- User Stories 2 and 3 are validation/error handling tasks that can be done in parallel

