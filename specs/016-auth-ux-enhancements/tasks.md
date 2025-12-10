# Tasks: Enhanced Authentication UX and Personalization

**Input**: Design documents from `/specs/016-auth-ux-enhancements/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Tests are OPTIONAL - not explicitly requested in specification, so test tasks are excluded.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `Auth/src/` (Node.js/Express service)
- **Frontend**: `frontend/src/` (Docusaurus/React)
- **Database**: `Auth/src/db/migrations/` (SQL migrations)

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 [P] Create directory structure for avatar components in src/components/Avatar/
- [X] T002 [P] Create directory structure for avatar routes in Auth/src/routes/
- [X] T003 [P] Create directory structure for avatar services in Auth/src/services/
- [X] T004 [P] Create static avatars directory in static/avatars/
- [X] T005 [P] Install Better Auth client SDK in frontend: `npm install better-auth` (already installed in root package.json)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Create database migration file Auth/src/db/migrations/002-avatar-personalization.sql with Avatar table schema
- [X] T007 [P] Add INSERT statements for 10 predefined avatars in Auth/src/db/migrations/002-avatar-personalization.sql
- [X] T008 [P] Add new columns to user_profile table in Auth/src/db/migrations/002-avatar-personalization.sql (selectedAvatarId, softwarePreferences, hardwarePreferences, programmingLanguagePreferences)
- [X] T009 [P] Add indexes for Avatar table and JSONB columns in Auth/src/db/migrations/002-avatar-personalization.sql
- [X] T010 [P] Add foreign key constraint for selectedAvatarId in Auth/src/db/migrations/002-avatar-personalization.sql
- [ ] T011 Run database migration: `cd Auth && npm run migrate:custom` to apply 002-avatar-personalization.sql (NOTE: Requires database connection - run manually)
- [X] T012 [P] Update TypeScript schema types in Auth/src/db/schema.ts to include Avatar interface
- [X] T013 [P] Update TypeScript schema types in Auth/src/db/schema.ts to extend UserProfile interface with new fields
- [ ] T014 [P] Create 10 avatar image files (200x200px PNG) and place in static/avatars/ (avatar-1.png through avatar-10.png) (NOTE: Requires image creation - can use placeholders)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Avatar Display and Navigation (Priority: P1) 🎯 MVP

**Goal**: Signed-in users see their avatar in navigation, sign-up/sign-in buttons are hidden, clicking avatar opens dropdown menu with Profile/Settings/Logout options.

**Independent Test**: Sign in with account that has selected avatar → verify avatar displays and buttons hidden. Sign in without avatar → verify first-letter avatar displays. Click avatar → verify dropdown opens with correct menu items.

### Implementation for User Story 1

- [X] T015 [P] [US1] Create AvatarService class in Auth/src/services/avatar-service.ts with getActiveAvatars method
- [X] T016 [P] [US1] Create AvatarService class in Auth/src/services/avatar-service.ts with getUserAvatarData method
- [X] T017 [P] [US1] Create avatar-generator utility in Auth/src/utils/avatar-generator.ts with extractFirstLetter function
- [X] T018 [US1] Create avatar routes file Auth/src/routes/avatar.ts with GET /avatar/list endpoint (public, returns all active avatars)
- [X] T019 [US1] Create avatar routes file Auth/src/routes/avatar.ts with GET /personalization/avatar endpoint (protected, returns user's avatar data)
- [X] T020 [US1] Mount avatar routes in Auth/src/index.ts by importing and using router at /api/avatar and /api/personalization
- [X] T021 [P] [US1] Create useAuth hook in src/hooks/useAuth.ts that wraps Better Auth client SDK for session management
- [X] T022 [P] [US1] Create FirstLetterAvatar component in src/components/Avatar/FirstLetterAvatar.tsx that generates SVG avatar with first letter
- [X] T023 [P] [US1] Create AvatarDropdown component in src/components/Avatar/AvatarDropdown.tsx with Profile, Settings, Logout menu items
- [X] T024 [US1] Create Avatar component in src/components/Avatar/Avatar.tsx that displays selected avatar or first-letter avatar, handles click to toggle dropdown
- [X] T025 [US1] Implement outside-click detection in src/components/Avatar/AvatarDropdown.tsx using useRef and useEffect hooks
- [X] T026 [US1] Implement dropdown toggle behavior in src/components/Avatar/Avatar.tsx (opens if closed, closes if open)
- [X] T027 [US1] Add navigation handlers in src/components/Avatar/AvatarDropdown.tsx for Profile (navigate to /profile), Settings (navigate to /profile#settings or scroll to settings section), Logout (call signOut, redirect to /)
- [X] T028 [US1] Implement auto-close dropdown after menu item selection in src/components/Avatar/AvatarDropdown.tsx
- [X] T029 [US1] Create AuthNav component integration in src/client-modules/navbar-inject.tsx that conditionally shows avatar (when signed in) or sign-up/sign-in buttons (when signed out)
- [X] T030 [US1] Update Docusaurus Navbar via client module src/client-modules/navbar-inject.tsx to use Avatar component for authentication-aware navigation
- [X] T031 [US1] Add API call in src/components/Avatar/Avatar.tsx to fetch user avatar data from GET /api/personalization/avatar on component mount
- [X] T032 [US1] Handle edge cases in src/components/Avatar/FirstLetterAvatar.tsx for empty names, special characters, non-ASCII characters (handled in Avatar.tsx and avatar-generator.ts)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Improved Sign-Up Error Handling (Priority: P1)

**Goal**: When sign-up fails due to existing email, user sees error message and remains on sign-up page (no automatic redirect). On successful sign-up, user is automatically signed in and redirected to dashboard.

**Independent Test**: Attempt sign-up with existing email → verify error message appears, user stays on sign-up page. Attempt sign-up with new email → verify account created, user automatically signed in, redirected to dashboard.

### Implementation for User Story 2

- [X] T033 [US2] Update sign-up form in src/components/Auth/SignUp.tsx to handle Better Auth sign-up response and check for error field
- [X] T034 [US2] Implement error message display in src/components/Auth/SignUp.tsx that shows inline error when account already exists, keeps user on page
- [X] T035 [US2] Remove automatic redirect on sign-up error in src/components/Auth/SignUp.tsx (only redirect on success)
- [X] T036 [US2] Implement auto sign-in after successful sign-up in src/components/Auth/SignUp.tsx (Better Auth handles this, verify session is created)
- [X] T037 [US2] Add redirect to dashboard after successful sign-up in src/components/Auth/SignUp.tsx (window.location.href = '/dashboard')
- [X] T038 [US2] Add link to sign-in page in error message in src/components/Auth/SignUp.tsx when account already exists
- [X] T039 [US2] Ensure form remains functional after error in src/components/Auth/SignUp.tsx (user can change email and resubmit)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Improved Sign-In Error Handling (Priority: P1)

**Goal**: When sign-in fails, user sees error message and remains on sign-in page (no automatic redirect). On successful sign-in, user is redirected to dashboard.

**Independent Test**: Attempt sign-in with wrong credentials → verify error message appears, user stays on sign-in page. Attempt sign-in with correct credentials → verify user signed in, redirected to dashboard.

### Implementation for User Story 3

- [X] T040 [US3] Update sign-in form in src/components/Auth/SignIn.tsx to handle Better Auth sign-in response and check for error field
- [X] T041 [US3] Implement error message display in src/components/Auth/SignIn.tsx that shows inline error when credentials are invalid, keeps user on page
- [X] T042 [US3] Remove automatic redirect on sign-in error in src/components/Auth/SignIn.tsx (only redirect on success)
- [X] T043 [US3] Add redirect to dashboard after successful sign-in in src/components/Auth/SignIn.tsx (window.location.href = '/dashboard')
- [X] T044 [US3] Ensure error message in src/components/Auth/SignIn.tsx does not reveal whether email exists or password is wrong (generic "Invalid email or password" message)
- [X] T045 [US3] Add "Forgot Password" link in sign-in error message in src/components/Auth/SignIn.tsx
- [X] T046 [US3] Add link to sign-up page in error message in src/components/Auth/SignIn.tsx
- [X] T047 [US3] Ensure form remains functional after error in src/components/Auth/SignIn.tsx (user can correct credentials and resubmit)

**Checkpoint**: At this point, User Stories 1, 2, AND 3 should all work independently

---

## Phase 6: User Story 4 - Enhanced Sign-Up Form with Personalization Questions (Priority: P2)

**Goal**: Sign-up form includes optional avatar selection and three personalization questions (software, hardware, programming languages) with multi-select checkboxes. All data saved during account creation.

**Independent Test**: Complete sign-up form with avatar and preferences → verify account created, avatar saved, preferences saved. Complete sign-up without avatar/preferences → verify account still created with null values.

### Implementation for User Story 4

- [X] T048 [P] [US4] Create PUT /personalization/avatar endpoint in Auth/src/routes/avatar.ts (protected, updates user's selectedAvatarId)
- [X] T049 [P] [US4] Create PUT /personalization/preferences endpoint in Auth/src/routes/personalization.ts (protected, updates user's preference JSONB fields)
- [X] T050 [US4] Add updateUserAvatar method to AvatarService in Auth/src/services/avatar-service.ts
- [X] T051 [US4] Add updateUserPreferences method to personalization routes in Auth/src/routes/personalization.ts (or create PreferencesService)
- [X] T052 [US4] Add validation in Auth/src/routes/avatar.ts PUT endpoint to ensure avatarId references active avatar
- [X] T053 [US4] Add validation in Auth/src/routes/personalization.ts PUT endpoint to ensure preference arrays contain only strings
- [X] T054 [P] [US4] Create avatar selection UI component in src/components/Auth/SignUp.tsx showing grid of 10 avatars with selection state
- [X] T055 [P] [US4] Create personalization questions UI in src/components/Auth/SignUp.tsx with three multi-select checkbox groups (software, hardware, programming languages)
- [X] T056 [US4] Define predefined option lists for software preferences in src/components/Auth/SignUp.tsx (e.g., ["ROS 2", "Gazebo", "Python", "ROS", "Gazebo Sim", "RViz", "MoveIt", "OpenCV"])
- [X] T057 [US4] Define predefined option lists for hardware preferences in src/components/Auth/SignUp.tsx (e.g., ["Humanoid robots", "Manipulators", "Sensors", "Actuators", "Mobile robots", "Drones"])
- [X] T058 [US4] Define predefined option lists for programming language preferences in src/components/Auth/SignUp.tsx (e.g., ["Python", "C++", "JavaScript", "TypeScript", "Rust", "Go"])
- [X] T059 [US4] Integrate avatar selection and personalization questions into sign-up form in src/components/Auth/SignUp.tsx (all fields on same form)
- [X] T060 [US4] Update sign-up form submission handler in src/components/Auth/SignUp.tsx to save avatar selection via PUT /api/personalization/avatar after account creation
- [X] T061 [US4] Update sign-up form submission handler in src/components/Auth/SignUp.tsx to save preferences via PUT /api/personalization/preferences after account creation
- [X] T062 [US4] Handle optional fields in sign-up form in src/components/Auth/SignUp.tsx (avatar and preferences can be null/empty, account still created)
- [X] T063 [US4] Add GET /personalization/profile endpoint in Auth/src/routes/personalization.ts (protected, returns complete user profile with avatar and preferences)
- [X] T064 [US4] Create profile page in src/pages/profile.tsx with profile section (displays user info, avatar, preferences) and settings section (allows editing avatar selection and personalization preferences)
- [X] T065 [US4] Add avatar selection UI in src/pages/profile.tsx settings section for changing avatar after sign-up
- [X] T066 [US4] Add personalization preferences editing UI in src/pages/profile.tsx settings section with same multi-select checkbox format as sign-up

**Checkpoint**: At this point, all user stories should be independently functional

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T067 [P] Add loading states for avatar data fetching in src/components/Avatar/Avatar.tsx
- [X] T068 [P] Add error handling for avatar API calls in src/components/Avatar/Avatar.tsx
- [X] T069 [P] Add loading states for sign-up/sign-in form submissions in src/components/Auth/SignUp.tsx and src/components/Auth/SignIn.tsx
- [X] T070 [P] Add accessibility attributes (aria-expanded, aria-haspopup) to avatar dropdown in src/components/Avatar/AvatarDropdown.tsx
- [X] T071 [P] Add keyboard navigation support for avatar dropdown in src/components/Avatar/AvatarDropdown.tsx (Enter to open, Escape to close, Arrow keys to navigate)
- [X] T072 [P] Add error logging for avatar service operations in Auth/src/services/avatar-service.ts
- [X] T073 [P] Add error logging for preference updates in Auth/src/routes/personalization.ts
- [X] T074 [P] Update documentation in Auth/AUTH_UX_FEATURES.md documenting new avatar and personalization features
- [ ] T075 [P] Validate quickstart.md scenarios by manually testing all user flows (NOTE: Requires manual testing)
- [X] T076 [P] Add responsive design considerations for avatar dropdown on mobile devices in src/components/Avatar/AvatarDropdown.tsx
- [X] T077 [P] Add visual feedback for avatar selection in sign-up form (highlight selected avatar)
- [X] T078 [P] Add visual feedback for dropdown menu item hover states in src/components/Avatar/AvatarDropdown.tsx

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Independent, only affects sign-up page
- **User Story 3 (P1)**: Can start after Foundational (Phase 2) - Independent, only affects sign-in page
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - May use avatar endpoints from US1 but can be implemented independently

### Within Each User Story

- Models/entities before services
- Services before endpoints/routes
- Backend endpoints before frontend components
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Models/services within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members
- Frontend components marked [P] can be developed in parallel

---

## Parallel Example: User Story 1

```bash
# Launch all backend services for User Story 1 together:
Task: "Create AvatarService class in Auth/src/services/avatar-service.ts with getActiveAvatars method"
Task: "Create avatar-generator utility in Auth/src/utils/avatar-generator.ts with extractFirstLetter function"

# Launch all frontend components for User Story 1 together:
Task: "Create useAuth hook in frontend/src/hooks/useAuth.ts"
Task: "Create FirstLetterAvatar component in frontend/src/components/Avatar/FirstLetterAvatar.tsx"
Task: "Create AvatarDropdown component in frontend/src/components/Avatar/AvatarDropdown.tsx"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Avatar Display and Navigation)
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Avatar Display)
   - Developer B: User Story 2 (Sign-Up Error Handling)
   - Developer C: User Story 3 (Sign-In Error Handling)
3. After P1 stories complete:
   - Developer A: User Story 4 (Personalization Questions)
   - Developer B/C: Polish & Cross-Cutting Concerns
4. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- Avatar images must be created and placed in frontend/static/avatars/ before testing
- Better Auth handles auto sign-in natively - no custom implementation needed
- All preference fields are optional - users can skip during sign-up

