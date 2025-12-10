# Tasks: User Authentication and Content Personalization

**Input**: Design documents from `/specs/015-auth-personalization/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Tests are OPTIONAL - not included as spec does not request TDD approach.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create Auth folder structure `Auth/src/` at repository root
- [x] T002 [P] Initialize Node.js project in `Auth/` folder with `npm init -y`
- [x] T003 [P] Create TypeScript configuration `Auth/tsconfig.json` per plan.md structure
- [x] T004 [P] Create `.env.example` file in `Auth/` with required environment variables (DATABASE_URL, BETTER_AUTH_SECRET, BETTER_AUTH_URL, FRONTEND_URL, EMAIL_SERVER)
- [x] T005 [P] Create `Auth/package.json` with dependencies: express, better-auth, cors, dotenv, and dev dependencies: @types/express, @types/cors, @types/node, typescript, ts-node
- [x] T006 [P] Create `Auth/vercel.json` for Vercel deployment configuration
- [x] T007 [P] Create frontend auth client file `src/lib/auth-client.ts` with Better Auth React client configuration using `useDocusaurusContext().siteConfig.customFields.authUrl` for baseURL
- [x] T008 [P] Create frontend component directories `src/components/Auth/` and `src/components/Personalization/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T009 Provision Neon Serverless Postgres database and obtain connection string
- [ ] T010 Create `Auth/.env` file with DATABASE_URL from Neon (add to .gitignore)
- [ ] T011 Generate BETTER_AUTH_SECRET using `node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"` and add to `Auth/.env`
- [ ] T012 Configure `Auth/.env` with BETTER_AUTH_URL (local: http://localhost:3000, production: Vercel URL)
- [ ] T013 Configure `Auth/.env` with FRONTEND_URL (local: http://localhost:3001, production: GitHub Pages URL)
- [x] T014 Create Better Auth configuration file `Auth/src/auth.ts` with PostgreSQL adapter and email/password enabled
- [x] T015 Create Express server entry point `Auth/src/index.ts` with CORS middleware and Better Auth handler mounted at `/api/auth/*`
- [ ] T016 Run Better Auth migrations `npx @better-auth/cli migrate` in `Auth/` folder to create auth tables
- [x] T017 Create database migration script `Auth/src/db/migrations/001-custom-tables.sql` for custom personalization tables (user_profile, reading_progress, bookmark, user_note, user_comment, chat_session, downloadable_resource, download_history, module_recommendation) per data-model.md
- [ ] T018 Execute custom tables migration against Neon database
- [x] T019 Create database schema types file `Auth/src/db/schema.ts` with TypeScript types for custom tables
- [x] T020 Create personalization API routes file `Auth/src/routes/personalization.ts` with Express router setup
- [x] T021 Create authentication middleware `Auth/src/utils/auth-middleware.ts` to extract user from Better Auth session
- [ ] T022 Test Auth service locally: start server with `npm run dev` and verify `/health` endpoint responds
- [ ] T023 Test Better Auth endpoints: verify `/api/auth/sign-up/email` and `/api/auth/sign-in/email` endpoints are accessible
- [ ] T024 Configure auth URL for Docusaurus: create `.env` with `AUTH_URL=http://localhost:3000`, then in `docusaurus.config.ts` add `customFields: { authUrl: process.env.AUTH_URL || 'http://localhost:3000' }` to make it available via `useDocusaurusContext().siteConfig.customFields.authUrl`

**Checkpoint**: Foundation ready - Auth service running locally, database provisioned and migrated, Better Auth configured, user story implementation can now begin

---

## Phase 3: User Story 1 - Account Creation and Authentication (Priority: P1) 🎯 MVP

**Goal**: Enable users to create accounts and sign in to access personalized features. Users can sign up with email/password, sign in, and maintain authenticated sessions across page navigations.

**Independent Test**: Create a new account via sign-up page, verify account is created and user is automatically signed in. Sign out, then sign in with credentials. Navigate between pages and verify session persists. Test error handling for invalid credentials and duplicate emails.

### Implementation for User Story 1

- [x] T025 [US1] Create SignUp component `src/components/Auth/SignUp.tsx` with email, password, and name input fields
- [x] T026 [US1] Create SignIn component `src/components/Auth/SignIn.tsx` with email and password input fields
- [x] T027 [US1] Implement sign-up form submission in `src/components/Auth/SignUp.tsx` using `authClient.signUp.email()` with error handling
- [x] T028 [US1] Implement sign-in form submission in `src/components/Auth/SignIn.tsx` using `authClient.signIn.email()` with error handling
- [x] T029 [US1] Create sign-up page `src/pages/signup.tsx` using Docusaurus Layout wrapper and SignUp component
- [x] T030 [US1] Create sign-in page `src/pages/signin.tsx` using Docusaurus Layout wrapper and SignIn component
- [x] T031 [US1] Create AuthProvider component `src/components/Auth/AuthProvider.tsx` to wrap app with Better Auth context
- [x] T032 [US1] Create useAuth hook `src/components/Auth/useAuth.ts` to access Better Auth session and user data
- [x] T033 [US1] Add sign-up link to homepage or navbar in `docusaurus.config.ts` or `src/pages/index.tsx`
- [x] T034 [US1] Add sign-in link to homepage or navbar in `docusaurus.config.ts` or `src/pages/index.tsx`
- [x] T035 [US1] Implement sign-out functionality in `src/components/Auth/AuthProvider.tsx` using `authClient.signOut()`
- [x] T036 [US1] Create user profile component `src/components/Auth/UserProfile.tsx` to display user name and email
- [x] T037 [US1] Add user profile display to navbar when authenticated (update `docusaurus.config.ts` or create custom navbar component)
- [x] T038 [US1] Implement session persistence check on page load in `src/components/Auth/AuthProvider.tsx` using `authClient.getSession()`
- [x] T039 [US1] Handle authentication errors gracefully: display user-friendly error messages in SignUp and SignIn components
- [x] T040 [US1] Implement password validation in `src/components/Auth/SignUp.tsx` (minimum length, complexity requirements)
- [x] T041 [US1] Implement email validation in `src/components/Auth/SignUp.tsx` (format validation)
- [ ] T042 [US1] Test sign-up flow: create account, verify automatic sign-in, check session cookie is set
- [ ] T043 [US1] Test sign-in flow: sign out, sign in with credentials, verify session is restored
- [ ] T044 [US1] Test session persistence: sign in, navigate between pages, verify user remains authenticated
- [ ] T045 [US1] Test error handling: attempt sign-up with duplicate email, verify error message displays
- [ ] T046 [US1] Test error handling: attempt sign-in with incorrect credentials, verify error message displays

**Checkpoint**: At this point, User Story 1 should be fully functional - users can create accounts, sign in, sign out, and maintain sessions. Authentication foundation is complete for all subsequent features.

---

## Phase 4: User Story 2 - Reading Progress Tracking (Priority: P2)

**Goal**: Automatically track which sections users have viewed and calculate progress percentages per module. Display progress in dashboard and on module pages.

**Independent Test**: Sign in, navigate through multiple sections across different modules, verify progress is tracked. Visit dashboard and verify progress percentages are displayed correctly. Return to a module overview page and verify visual indicators show completed sections.

### Implementation for User Story 2

- [x] T047 [US2] Create progress tracking service `src/components/Personalization/services/progressService.ts` with functions to record and fetch progress
- [x] T048 [US2] Create useProgressTracking hook `src/components/Personalization/hooks/useProgressTracking.ts` to track section views on page load
- [x] T049 [US2] Implement POST endpoint `/api/personalization/progress` in `Auth/src/routes/personalization.ts` to record section views
- [x] T050 [US2] Implement GET endpoint `/api/personalization/progress` in `Auth/src/routes/personalization.ts` to fetch user progress with summary calculations
- [x] T051 [US2] Create ProgressIndicator component `src/components/Personalization/ProgressIndicator.tsx` to display progress percentage for a module
- [ ] T052 [US2] Integrate progress tracking into module section pages: add useEffect hook to call `useProgressTracking()` when page loads
- [x] T053 [US2] Create progress calculation utility `Auth/src/utils/progress-calculator.ts` to compute progress percentages from reading_progress table
- [x] T054 [US2] Implement progress summary endpoint logic in `Auth/src/routes/personalization.ts` to return progress by module with percentages
- [x] T055 [US2] Add progress display to dashboard: fetch and display progress for all modules user has started
- [x] T056 [US2] Create module progress visualization component `src/components/Personalization/ModuleProgress.tsx` to show progress bars per module
- [ ] T057 [US2] Add progress indicators to module overview pages: display which sections are completed
- [ ] T058 [US2] Implement duplicate view prevention: ensure same section viewed multiple times only counts once in progress calculation
- [ ] T059 [US2] Test progress tracking: view multiple sections, verify progress is recorded in database
- [ ] T060 [US2] Test progress calculation: verify percentages are calculated correctly (viewed sections / total sections)
- [ ] T061 [US2] Test progress display: verify dashboard shows accurate progress for all modules
- [ ] T062 [US2] Test progress indicators: verify module pages show visual indicators for completed sections

**Checkpoint**: At this point, User Story 2 should be complete - progress is automatically tracked, calculated, and displayed in dashboard and module pages.

---

## Phase 5: User Story 3 - Bookmarks and Favorites (Priority: P2)

**Goal**: Allow authenticated users to bookmark sections for quick access. Unauthenticated users are prompted to sign in when attempting to bookmark.

**Independent Test**: As unauthenticated user, attempt to bookmark a section, verify sign-in prompt appears. Sign in, bookmark multiple sections, verify bookmarks are saved. Visit dashboard and verify bookmarked sections are listed. Click bookmark button again to remove bookmark.

### Implementation for User Story 3

- [x] T063 [US3] Create BookmarkButton component `src/components/Personalization/BookmarkButton.tsx` with bookmark/unbookmark functionality
- [x] T064 [US3] Create bookmark service `src/components/Personalization/services/bookmarkService.ts` with functions to create, delete, and fetch bookmarks
- [x] T065 [US3] Implement POST endpoint `/api/personalization/bookmarks` in `Auth/src/routes/personalization.ts` to create bookmarks
- [x] T066 [US3] Implement DELETE endpoint `/api/personalization/bookmarks/:id` in `Auth/src/routes/personalization.ts` to delete bookmarks
- [x] T067 [US3] Implement GET endpoint `/api/personalization/bookmarks` in `Auth/src/routes/personalization.ts` to fetch user bookmarks
- [x] T068 [US3] Implement GET endpoint `/api/personalization/bookmarks/check` in `Auth/src/routes/personalization.ts` to check if section is bookmarked
- [ ] T069 [US3] Add BookmarkButton to module section pages: integrate into page layout
- [x] T070 [US3] Implement bookmark state management in `src/components/Personalization/BookmarkButton.tsx` to show bookmarked/unbookmarked state
- [x] T071 [US3] Implement sign-in prompt in `src/components/Personalization/BookmarkButton.tsx` when unauthenticated user clicks bookmark
- [x] T072 [US3] Create bookmarks list component `src/components/Personalization/BookmarksList.tsx` to display all user bookmarks
- [x] T073 [US3] Add bookmarks section to dashboard: display bookmarks with links to sections
- [x] T074 [US3] Implement bookmark organization: group bookmarks by module in `src/components/Personalization/BookmarksList.tsx`
- [ ] T075 [US3] Test bookmark creation: sign in, bookmark a section, verify bookmark is saved in database
- [ ] T076 [US3] Test bookmark removal: remove bookmark, verify it's deleted from database
- [ ] T077 [US3] Test bookmark state: verify bookmark button shows correct state when page loads
- [ ] T078 [US3] Test unauthenticated bookmark: attempt to bookmark without signing in, verify sign-in prompt
- [ ] T079 [US3] Test bookmarks display: verify dashboard shows all bookmarked sections correctly

**Checkpoint**: At this point, User Story 3 should be complete - users can bookmark sections, view bookmarks in dashboard, and unauthenticated users are prompted to sign in.

---

## Phase 6: User Story 4 - Personalized Module Recommendations (Priority: P3)

**Goal**: Generate and display personalized module recommendations based on user progress, learning level, and completion status.

**Independent Test**: Complete Module 1, visit dashboard, verify recommendation to start Module 2 appears. Start but don't complete Module 2, verify recommendation to continue Module 2 appears. Set learning level to beginner, verify recommendations prioritize foundational modules.

### Implementation for User Story 4

- [x] T080 [US4] Create recommendation service `Auth/src/services/recommendation-service.ts` to generate recommendations based on progress
- [x] T081 [US4] Implement recommendation algorithm in `Auth/src/services/recommendation-service.ts`: next module in sequence, continue incomplete modules, consider learning level
- [x] T082 [US4] Implement GET endpoint `/api/personalization/recommendations` in `Auth/src/routes/personalization.ts` to return personalized recommendations
- [x] T083 [US4] Create Recommendations component `src/components/Personalization/Recommendations.tsx` to display recommended modules
- [x] T084 [US4] Add recommendations section to dashboard: fetch and display recommendations
- [x] T085 [US4] Implement learning level consideration: factor user's learning level into recommendation priority
- [ ] T086 [US4] Test recommendations: complete Module 1, verify Module 2 is recommended
- [ ] T087 [US4] Test recommendations: start Module 2, verify continue Module 2 is recommended
- [ ] T088 [US4] Test recommendations: set learning level, verify recommendations prioritize appropriate modules

**Checkpoint**: At this point, User Story 4 should be complete - personalized recommendations are generated and displayed in dashboard.

---

## Phase 7: User Story 5 - Chatbot Session Integration (Priority: P2)

**Goal**: Link chatbot sessions to user accounts, save chat history, and enable personalized responses based on user progress.

**Independent Test**: Sign in, have a conversation with chatbot, verify session is linked to user account. Sign out and back in, verify chat history is accessible. Ask chatbot about concepts from modules user has completed, verify personalized responses.

### Implementation for User Story 5

- [x] T089 [US5] Extend existing chatbot service `src/components/ChatWidget/services/chatService.ts` to include user ID in requests when authenticated
- [ ] T090 [US5] Update FastAPI chatbot backend to accept user_id parameter in chat requests (modify `Chatbot/src/chatbot/agent/agent.py` or relevant endpoint)
- [x] T091 [US5] Create chat session storage endpoint `POST /api/personalization/chat-sessions` in `Auth/src/routes/personalization.ts` to save chat sessions
- [x] T092 [US5] Create chat history retrieval endpoint `GET /api/personalization/chat-sessions` in `Auth/src/routes/personalization.ts` to fetch user's chat history
- [ ] T093 [US5] Update chatbot to send user context: include user progress and bookmarks in chat requests for personalized responses
- [x] T094 [US5] Create chat history component `src/components/Personalization/ChatHistory.tsx` to display previous conversations
- [ ] T095 [US5] Integrate chat history into chatbot widget: show previous conversations when user opens chatbot
- [ ] T096 [US5] Update chatbot backend to use user progress data: modify agent to reference user's completed modules in responses
- [ ] T097 [US5] Test chat session linking: sign in, chat with bot, verify session is saved with user_id
- [ ] T098 [US5] Test chat history: sign out and back in, verify previous conversations are accessible
- [ ] T099 [US5] Test personalized responses: complete Module 1, ask about Module 2, verify chatbot references progress

**Checkpoint**: At this point, User Story 5 should be complete - chatbot sessions are linked to users, history is preserved, and responses are personalized.

---

## Phase 8: User Story 6 - User Notes and Annotations (Priority: P3)

**Goal**: Allow authenticated users to add personal notes to any section. Notes are saved to user account and displayed on section pages and dashboard.

**Independent Test**: Sign in, add a note to a section, verify note is saved. Return to section, verify note is displayed. Edit note, verify update is saved. Delete note, verify it's removed. View dashboard, verify all notes are listed.

### Implementation for User Story 6

- [x] T100 [US6] Create NoteEditor component `src/components/Personalization/NoteEditor.tsx` with create, edit, and delete functionality
- [x] T101 [US6] Create note service `src/components/Personalization/services/noteService.ts` with functions to create, update, delete, and fetch notes
- [x] T102 [US6] Implement POST endpoint `/api/personalization/notes` in `Auth/src/routes/personalization.ts` to create or update notes
- [x] T103 [US6] Implement DELETE endpoint `/api/personalization/notes/:id` in `Auth/src/routes/personalization.ts` to delete notes
- [x] T104 [US6] Implement GET endpoint `/api/personalization/notes` in `Auth/src/routes/personalization.ts` to fetch user notes
- [ ] T105 [US6] Add NoteEditor to module section pages: integrate into page layout
- [x] T106 [US6] Implement note display on section pages: fetch and display user's note for current section
- [x] T107 [US6] Create notes list component `src/components/Personalization/NotesList.tsx` to display all user notes
- [x] T108 [US6] Add notes section to dashboard: display notes with links to sections
- [x] T109 [US6] Implement sign-in prompt in `src/components/Personalization/NoteEditor.tsx` when unauthenticated user attempts to add note
- [ ] T110 [US6] Test note creation: sign in, add note, verify note is saved in database
- [ ] T111 [US6] Test note update: edit existing note, verify update is saved
- [ ] T112 [US6] Test note deletion: delete note, verify it's removed from database
- [ ] T113 [US6] Test note display: verify notes appear on section pages and dashboard
- [ ] T114 [US6] Test unauthenticated note: attempt to add note without signing in, verify sign-in prompt

**Checkpoint**: At this point, User Story 6 should be complete - users can add, edit, delete, and view notes on sections and dashboard.

---

## Phase 9: User Story 7 - Downloadable Resources for Authenticated Users (Priority: P2)

**Goal**: Provide downloadable resources (PDFs, code examples, datasets) that are only accessible to authenticated users. Track download history.

**Independent Test**: As unauthenticated user, attempt to download resource, verify redirect to sign-in. Sign in, navigate to resources page, view available downloads, download a file, verify download succeeds. Check dashboard, verify download history is recorded.

### Implementation for User Story 7

- [ ] T115 [US7] Set up cloud storage service (Vercel Blob, AWS S3, or Cloudflare R2) for downloadable resources
- [ ] T116 [US7] Create downloadable resources management: add initial resources to `downloadable_resource` table via migration or admin script
- [x] T117 [US7] Implement GET endpoint `/api/personalization/resources` in `Auth/src/routes/personalization.ts` to list available resources
- [x] T118 [US7] Implement POST endpoint `/api/personalization/resources/:id/download` in `Auth/src/routes/personalization.ts` to generate signed download URLs
- [ ] T119 [US7] Create signed URL generation utility `Auth/src/utils/signed-url.ts` to create time-limited download URLs
- [x] T120 [US7] Implement download tracking: record downloads in `download_history` table when user downloads resource
- [x] T121 [US7] Create ResourcesPage component `src/pages/resources.tsx` to display available downloadable resources
- [x] T122 [US7] Implement resource download functionality: fetch signed URL and initiate download
- [x] T123 [US7] Implement authentication check: redirect unauthenticated users to sign-in when accessing download links
- [x] T124 [US7] Create download history component `src/components/Personalization/DownloadHistory.tsx` to display user's download history
- [x] T125 [US7] Add download history section to dashboard: display downloaded resources
- [x] T126 [US7] Implement resource organization: group resources by module in `src/pages/resources.tsx`
- [ ] T127 [US7] Test resource listing: sign in, verify resources page shows available downloads
- [ ] T128 [US7] Test resource download: download a file, verify download succeeds and history is recorded
- [ ] T129 [US7] Test unauthenticated download: attempt to download without signing in, verify redirect to sign-in
- [ ] T130 [US7] Test download history: verify dashboard shows download history correctly

**Checkpoint**: At this point, User Story 7 should be complete - authenticated users can download resources, download history is tracked, and unauthenticated users are redirected to sign in.

---

## Phase 10: User Story 8 - User Comments and Discussions (Priority: P3)

**Goal**: Enable users to post comments on module sections. Unauthenticated users can view comments but cannot post. Authenticated users can post comments and reply to other users' comments.

**Independent Test**: As unauthenticated user, view comments section, verify comments are visible but no post form. Sign in, post a comment, verify comment appears. Reply to another user's comment, verify reply is nested. View dashboard, verify user's comments are listed.

### Implementation for User Story 8

- [x] T131 [US8] Create CommentSection component `src/components/Personalization/CommentSection.tsx` to display comments and comment form
- [x] T132 [US8] Create comment service `src/components/Personalization/services/commentService.ts` with functions to create, fetch, and reply to comments
- [x] T133 [US8] Implement POST endpoint `/api/personalization/comments` in `Auth/src/routes/personalization.ts` to create comments
- [x] T134 [US8] Implement GET endpoint `/api/personalization/comments` in `Auth/src/routes/personalization.ts` to fetch comments for a section (only approved comments)
- [ ] T135 [US8] Implement POST endpoint `/api/personalization/comments/:id/flag` in `Auth/src/routes/personalization.ts` to flag comments
- [ ] T136 [US8] Add CommentSection to module section pages: integrate into page layout
- [x] T137 [US8] Implement comment form display: show form only for authenticated users in `src/components/Personalization/CommentSection.tsx`
- [x] T138 [US8] Implement sign-in prompt: show sign-in prompt when unauthenticated user attempts to post comment
- [x] T139 [US8] Implement nested replies: support parentCommentId in comment creation for reply functionality
- [x] T140 [US8] Create comment moderation: implement basic moderation status (pending, approved, rejected, flagged) in database queries
- [x] T141 [US8] Implement comment display: show comments in chronological order with nested replies
- [x] T142 [US8] Create user comments list component `src/components/Personalization/UserCommentsList.tsx` to display user's comments
- [x] T143 [US8] Add user comments section to dashboard: display user's comments with links to sections
- [ ] T144 [US8] Test comment creation: sign in, post comment, verify comment is saved and displayed
- [ ] T145 [US8] Test comment replies: reply to a comment, verify reply is nested correctly
- [ ] T146 [US8] Test comment viewing: verify unauthenticated users can view approved comments
- [ ] T147 [US8] Test comment posting: verify unauthenticated users cannot post comments (sign-in prompt shown)
- [ ] T148 [US8] Test comment moderation: verify only approved comments are displayed

**Checkpoint**: At this point, User Story 8 should be complete - users can post comments, view comments, reply to comments, and unauthenticated users can view but not post.

---

## Phase 11: User Story 1 Extension - User Dashboard (Priority: P1)

**Goal**: Create a centralized dashboard where users can view their progress, bookmarks, notes, recommendations, download history, and comments.

**Independent Test**: Sign in, navigate to dashboard, verify all personalization data is displayed: progress percentages, bookmarks list, notes list, recommendations, download history, and user comments. Verify all links navigate to correct sections.

### Implementation for User Story 1 Extension

- [x] T149 [US1] Create Dashboard component `src/components/Personalization/Dashboard.tsx` to display all user personalization data
- [x] T150 [US1] Implement GET endpoint `/api/personalization/dashboard` in `Auth/src/routes/personalization.ts` to fetch all dashboard data in one request
- [x] T151 [US1] Create dashboard page `src/pages/dashboard.tsx` using Docusaurus Layout wrapper and Dashboard component
- [x] T152 [US1] Integrate progress display in `src/components/Personalization/Dashboard.tsx`: show progress percentages for all modules
- [x] T153 [US1] Integrate bookmarks display in `src/components/Personalization/Dashboard.tsx`: show bookmarked sections
- [x] T154 [US1] Integrate notes display in `src/components/Personalization/Dashboard.tsx`: show notes with links to sections
- [x] T155 [US1] Integrate recommendations display in `src/components/Personalization/Dashboard.tsx`: show personalized recommendations
- [x] T156 [US1] Integrate download history display in `src/components/Personalization/Dashboard.tsx`: show downloaded resources
- [x] T157 [US1] Integrate user comments display in `src/components/Personalization/Dashboard.tsx`: show user's comments
- [x] T158 [US1] Add dashboard link to navbar when user is authenticated
- [ ] T159 [US1] Test dashboard: sign in, visit dashboard, verify all sections display correctly
- [ ] T160 [US1] Test dashboard data: verify all personalization data is fetched and displayed accurately

**Checkpoint**: At this point, User Story 1 Extension should be complete - users have a centralized dashboard showing all personalization features.

---

## Phase 12: Polish & Cross-Cutting Concerns

**Purpose**: Final polish, error handling, performance optimization, and cross-cutting features

- [x] T161 Implement user profile update: allow users to update name and learning level via `PATCH /api/personalization/profile` endpoint
- [x] T162 Create user profile update UI in `src/components/Auth/UserProfile.tsx` with form to update name and learning level
- [ ] T163 Implement password reset flow: configure email service and test password reset functionality
- [ ] T164 Add loading states: implement loading indicators for all async operations (sign-up, sign-in, progress tracking, etc.)
- [ ] T165 Implement error boundaries: add React error boundaries to catch and display errors gracefully
- [ ] T166 Add retry logic: implement retry with exponential backoff for failed API requests
- [ ] T167 Optimize dashboard loading: implement data fetching optimization to load dashboard data efficiently
- [ ] T168 Add progress tracking debouncing: prevent excessive API calls when user navigates quickly
- [ ] T169 Implement offline handling: queue progress updates when offline, sync when connection restored
- [ ] T170 Add session expiration handling: detect expired sessions and prompt user to sign in again
- [ ] T171 Implement rate limiting: add rate limiting to personalization endpoints to prevent abuse
- [ ] T172 Add analytics tracking: track user engagement metrics (optional, for future analysis)
- [ ] T173 Test cross-device sync: sign in on multiple devices, verify progress and bookmarks sync
- [ ] T174 Test error scenarios: test network failures, server errors, invalid data handling
- [ ] T175 Performance testing: verify dashboard loads within 3 seconds (SC-006), progress updates within 5 seconds (SC-004)
- [ ] T176 Deploy Auth service to Vercel: configure production environment variables and deploy
- [ ] T177 Update frontend environment variables: set AUTH_URL in `.env` to production Vercel URL and update `docusaurus.config.ts` customFields.authUrl accordingly
- [ ] T178 Test production deployment: verify authentication and personalization features work in production
- [ ] T179 Update documentation: document authentication and personalization features in README or docs

**Checkpoint**: Feature is complete and production-ready - all user stories implemented, polished, tested, and deployed.

---

## Dependencies

### User Story Completion Order

1. **Phase 2 (Foundational)** - MUST complete before any user story
2. **Phase 3 (US1 - Authentication)** - Foundation for all other stories
3. **Phase 4 (US2 - Progress)** - Can be parallel with US3, US7
4. **Phase 5 (US3 - Bookmarks)** - Can be parallel with US2, US7
5. **Phase 7 (US5 - Chatbot)** - Depends on US1, can be parallel with US2, US3
6. **Phase 9 (US7 - Downloads)** - Can be parallel with US2, US3
7. **Phase 6 (US4 - Recommendations)** - Depends on US2 (progress tracking)
8. **Phase 8 (US6 - Notes)** - Can be parallel with US3, US8
9. **Phase 10 (US8 - Comments)** - Can be parallel with US6
10. **Phase 11 (US1 Extension - Dashboard)** - Depends on US2, US3, US4, US6, US7, US8
11. **Phase 12 (Polish)** - Depends on all user stories

### Parallel Execution Opportunities

**After Phase 2 (Foundational) completes**:
- US2 (Progress) + US3 (Bookmarks) + US7 (Downloads) can run in parallel
- US5 (Chatbot) can run in parallel with US2, US3, US7
- US6 (Notes) + US8 (Comments) can run in parallel

**After US2 (Progress) completes**:
- US4 (Recommendations) can begin (depends on progress data)

**After US2, US3, US4, US6, US7, US8 complete**:
- US1 Extension (Dashboard) can begin (aggregates all features)

## Implementation Strategy

### MVP Scope (Minimum Viable Product)

**Phase 2 (Foundational) + Phase 3 (US1 - Authentication)** = MVP

This delivers:
- User account creation and authentication
- Session management
- Foundation for all future personalization features

### Incremental Delivery

1. **MVP**: Authentication (Phase 2 + Phase 3)
2. **Increment 1**: Progress Tracking (Phase 4)
3. **Increment 2**: Bookmarks (Phase 5)
4. **Increment 3**: Downloads (Phase 9)
5. **Increment 4**: Chatbot Integration (Phase 7)
6. **Increment 5**: Recommendations (Phase 6)
7. **Increment 6**: Notes (Phase 8)
8. **Increment 7**: Comments (Phase 10)
9. **Increment 8**: Dashboard (Phase 11)
10. **Final**: Polish (Phase 12)

Each increment is independently testable and delivers value to users.

## Task Summary

- **Total Tasks**: 179
- **Setup Tasks**: 8 (Phase 1)
- **Foundational Tasks**: 16 (Phase 2)
- **User Story 1 Tasks**: 22 (Phase 3)
- **User Story 2 Tasks**: 16 (Phase 4)
- **User Story 3 Tasks**: 17 (Phase 5)
- **User Story 4 Tasks**: 9 (Phase 6)
- **User Story 5 Tasks**: 11 (Phase 7)
- **User Story 6 Tasks**: 15 (Phase 8)
- **User Story 7 Tasks**: 16 (Phase 9)
- **User Story 8 Tasks**: 18 (Phase 10)
- **User Story 1 Extension Tasks**: 12 (Phase 11)
- **Polish Tasks**: 19 (Phase 12)

## Format Validation

✅ All tasks follow the required checklist format:
- Checkbox: `- [ ]`
- Task ID: `T001`, `T002`, etc.
- Parallel marker: `[P]` where applicable
- Story label: `[US1]`, `[US2]`, etc. for user story tasks
- File paths included in descriptions

