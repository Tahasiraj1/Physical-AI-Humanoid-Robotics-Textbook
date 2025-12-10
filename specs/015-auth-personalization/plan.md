# Implementation Plan: User Authentication and Content Personalization

**Branch**: `015-auth-personalization` | **Date**: 2025-01-27 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/015-auth-personalization/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Add Better Auth sign-in/sign-up functionality with content personalization features to the Physical AI Humanoid Robotics Textbook. The implementation includes user authentication, progress tracking, bookmarks, notes, personalized recommendations, chatbot session integration, downloadable resources, and user comments. All book content remains accessible to unauthenticated users; authentication is only required for interactive personalization features.

**Technical Approach**: 
- Frontend: Docusaurus with `better-auth/react` for client-side authentication
- Backend: Separate Node.js/Express service in `Auth/` folder using Better Auth library
- Database: Neon Serverless Postgres for user data, progress, bookmarks, notes, comments
- Deployment: Auth service deployed separately on Vercel; Docusaurus continues on GitHub Pages
- Integration: CORS-enabled API communication between Docusaurus frontend and Vercel-deployed auth service

## Technical Context

**Language/Version**: TypeScript 5.6.2, Node.js >=20.0 (frontend), Node.js >=20.0 (backend)  
**Primary Dependencies**: 
- Frontend: Docusaurus 3.9.2, React 19.0.0, `better-auth/react`
- Backend: Express.js, `better-auth`, PostgreSQL adapter for Better Auth  
**Storage**: Neon Serverless Postgres database for all user data (accounts, progress, bookmarks, notes, comments, chat sessions, downloads)  
**Testing**: Jest/Vitest for unit tests, Playwright/Cypress for E2E tests, manual testing for authentication flows  
**Target Platform**: Web (Docusaurus static site on GitHub Pages, Auth service on Vercel), modern browsers  
**Project Type**: Web application (frontend + separate backend service)  
**Performance Goals**: 
- Sign-up completion in under 2 minutes (SC-001)
- Sign-in in under 30 seconds (SC-002)
- Progress tracking updates within 5 seconds (SC-004)
- Dashboard load in under 3 seconds (SC-006)
- Support 500 concurrent authenticated users (SC-013)  
**Constraints**: 
- Auth service must be separate from Docusaurus (deployed independently)
- All book content must remain accessible to unauthenticated users
- CORS must be configured for cross-origin requests
- Database must be serverless-compatible (Neon Postgres)
- Must maintain existing Docusaurus structure and functionality  
**Scale/Scope**: 
- Initial target: 500 concurrent authenticated users
- All 4 textbook modules with progress tracking
- Unlimited bookmarks, notes, and comments per user
- Chat history preservation across sessions

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Documentation-First Architecture ✅
- **Status**: PASS
- **Rationale**: Feature adds authentication and personalization without modifying existing documentation structure. All book content remains in markdown format. Personalization features enhance the reading experience but do not change content organization.

### II. Modular Content Organization ✅
- **Status**: PASS
- **Rationale**: Progress tracking, bookmarks, and notes reference existing module/section structure without modifying it. Module identifiers remain stable, enabling personalization data to reference content without coupling.

### III. Vector Database Integration ✅
- **Status**: PASS (No impact)
- **Rationale**: This feature does not modify the Qdrant embedding system or content embedding process. User personalization data is separate from content embeddings.

### IV. AI Agent Architecture ✅
- **Status**: PASS (Enhancement)
- **Rationale**: Chatbot integration enhances existing AI agent by linking sessions to user accounts and personalizing responses based on user progress. This extends rather than replaces existing agent functionality.

### V. Deployment Standards ✅
- **Status**: PASS
- **Rationale**: 
  - Docusaurus continues deploying to GitHub Pages (no change)
  - Auth service deploys separately to Vercel (new service, follows deployment standards)
  - Decoupled systems ensure book availability independent of auth service status

### VI. API-First Backend Design ✅
- **Status**: PASS
- **Rationale**: Auth service provides RESTful API endpoints via Better Auth. API contracts will be documented. Frontend consumes API independently, enabling future integrations.

**Overall Gate Status**: ✅ PASS - All constitution principles satisfied

## Project Structure

### Documentation (this feature)

```text
specs/015-auth-personalization/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   ├── auth-api.md      # Better Auth API endpoints
│   └── personalization-api.md  # Custom personalization endpoints
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Frontend (Docusaurus - existing structure)
src/
├── components/
│   ├── ChatWidget/          # Existing chatbot widget
│   ├── Auth/                 # NEW: Authentication components
│   │   ├── SignIn.tsx
│   │   ├── SignUp.tsx
│   │   ├── UserProfile.tsx
│   │   └── AuthProvider.tsx
│   ├── Personalization/     # NEW: Personalization components
│   │   ├── BookmarkButton.tsx
│   │   ├── NoteEditor.tsx
│   │   ├── ProgressIndicator.tsx
│   │   ├── CommentSection.tsx
│   │   └── Dashboard.tsx
│   └── HomepageFeatures/
├── pages/
│   ├── signin.tsx            # NEW: Sign-in page
│   ├── signup.tsx            # NEW: Sign-up page
│   ├── dashboard.tsx         # NEW: User dashboard
│   └── index.tsx
├── lib/
│   └── auth-client.ts        # NEW: Better Auth client configuration
└── css/

# Backend (Auth service - NEW separate folder)
Auth/
├── src/
│   ├── index.ts              # Express server entry point
│   ├── auth.ts               # Better Auth configuration
│   ├── routes/
│   │   ├── auth.ts           # Better Auth handler routes
│   │   └── personalization.ts  # Custom personalization endpoints
│   ├── db/
│   │   └── schema.ts         # Database schema (Better Auth + custom tables)
│   └── utils/
│       └── cors.ts            # CORS configuration
├── package.json
├── tsconfig.json
├── vercel.json               # Vercel deployment config
└── .env.example

# Existing structure (unchanged)
docs/                        # Book content (unchanged)
Chatbot/                     # Existing Python chatbot (unchanged)
```

**Structure Decision**: 
- **Frontend**: Extends existing Docusaurus structure with new auth and personalization components in `src/components/` and new pages in `src/pages/`
- **Backend**: New separate `Auth/` folder at repository root for Node.js/Express service, deployed independently to Vercel
- **Separation**: Clear separation between frontend (Docusaurus) and backend (Auth service) enables independent deployment and scaling

## Complexity Tracking

> **No constitution violations - all principles satisfied**

## Phase 0: Research & Discovery

**Status**: ✅ Complete

**Research Tasks**:
1. ✅ Better Auth integration patterns for React/Docusaurus
2. ✅ Better Auth Express.js server setup and deployment to Vercel
3. ✅ Neon Postgres database schema design for Better Auth + custom tables
4. ✅ CORS configuration for cross-origin Docusaurus ↔ Auth service communication
5. ✅ Progress tracking implementation patterns (page view detection, state management)
6. ✅ Chatbot session linking with user accounts (integration with existing FastAPI backend)
7. ✅ File storage for downloadable resources (cloud storage integration)
8. ✅ Comment moderation patterns and best practices

**Output**: ✅ `research.md` with all technical decisions and patterns

## Phase 1: Design & Contracts

**Status**: ✅ Complete

**Prerequisites**: ✅ `research.md` complete

**Design Tasks**:
1. ✅ Database schema design (`data-model.md`)
   - Better Auth default tables
   - Custom tables: progress, bookmarks, notes, comments, chat_sessions, downloads
   - Relationships and indexes
2. ✅ API contracts (`contracts/`)
   - Better Auth endpoints (documented in `contracts/auth-api.md`)
   - Custom personalization endpoints (documented in `contracts/personalization-api.md`)
   - Error handling and response formats
3. ✅ Quickstart guide (`quickstart.md`)
   - Local development setup
   - Database provisioning
   - Environment variables
   - Testing authentication flow

**Output**: ✅ `data-model.md`, `contracts/*.md`, `quickstart.md`

## Phase 2: Task Breakdown

**Prerequisites**: Phase 1 complete

**Note**: Task breakdown is handled by `/sp.tasks` command, not `/sp.plan`

**Output**: `tasks.md` (created by `/sp.tasks`)
