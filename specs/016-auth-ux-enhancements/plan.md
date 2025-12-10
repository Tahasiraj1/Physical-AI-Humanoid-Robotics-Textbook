# Implementation Plan: Enhanced Authentication UX and Personalization

**Branch**: `016-auth-ux-enhancements` | **Date**: 2025-12-08 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/016-auth-ux-enhancements/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This feature enhances the authentication user experience by adding avatar display with dropdown navigation, improved error handling that keeps users on sign-up/sign-in pages, automatic sign-in after sign-up, and personalization questions during account creation. The implementation extends the existing Better Auth service with new database tables (Avatar model), updated user profile schema, and frontend components for avatar display and dropdown menu.

**Primary Requirement**: Users see avatars when signed in, navigation buttons change based on auth state, sign-up automatically signs users in, and personalization data is collected during account creation.

**Technical Approach**: Extend existing Auth service (Node.js/Express/Better Auth) with new database migrations, API endpoints for avatar management, and frontend React components integrated into Docusaurus navigation.

## Technical Context

**Language/Version**: TypeScript 5.6.2, Node.js 20+  
**Primary Dependencies**: Better Auth 1.0.0, Express 4.18.2, PostgreSQL (pg 8.16.3), React (via Docusaurus)  
**Storage**: PostgreSQL (Neon Serverless) - existing user tables + new Avatar table + updated user_profile table  
**Testing**: Manual testing, Better Auth built-in validation, database migration scripts  
**Target Platform**: Web (Docusaurus frontend, Node.js/Express backend on Vercel)  
**Project Type**: Web application (frontend: Docusaurus, backend: Auth service)  
**Performance Goals**: Avatar display within 1 second, error messages within 2 seconds, form submissions under 3 seconds  
**Constraints**: Must work with existing Better Auth setup, backward compatible with existing user accounts, no breaking changes to existing API  
**Scale/Scope**: Supports existing user base, 10 predefined avatars, 3 personalization questions with multi-select options

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Gate 1: Documentation-First Architecture ✅
- **Status**: PASS
- **Rationale**: This feature enhances existing authentication system. No new documentation modules required. Frontend components are part of Docusaurus navigation (existing structure).

### Gate 2: Modular Content Organization ✅
- **Status**: PASS
- **Rationale**: Feature is self-contained within Auth service. No impact on book content modules.

### Gate 3: Vector Database Integration ✅
- **Status**: PASS
- **Rationale**: No impact on Qdrant or embedding pipeline. Personalization data may inform future recommendations but doesn't affect current RAG system.

### Gate 4: AI Agent Architecture ✅
- **Status**: PASS
- **Rationale**: No changes to AI Agent or FastAPI endpoints. Feature is isolated to authentication UX.

### Gate 5: Deployment Standards ✅
- **Status**: PASS
- **Rationale**: Uses existing Auth service deployment on Vercel. No new deployment targets required.

### Gate 6: API-First Backend Design ✅
- **Status**: PASS
- **Rationale**: New API endpoints follow existing RESTful patterns. Extends existing personalization API structure.

**Overall Gate Status**: ✅ ALL GATES PASS

## Project Structure

### Documentation (this feature)

```text
specs/016-auth-ux-enhancements/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
Auth/
├── src/
│   ├── auth.ts                    # Better Auth configuration (existing)
│   ├── index.ts                   # Express server (existing)
│   ├── db/
│   │   ├── migrations/
│   │   │   ├── 001-custom-tables.sql  # Existing migration
│   │   │   └── 002-avatar-personalization.sql  # NEW: Avatar + personalization fields
│   │   ├── migrate-custom-tables.ts   # Migration runner (existing)
│   │   └── schema.ts                  # TypeScript types (update)
│   ├── routes/
│   │   ├── personalization.ts      # Existing routes (update)
│   │   └── avatar.ts               # NEW: Avatar management endpoints
│   ├── services/
│   │   └── avatar-service.ts      # NEW: Avatar selection logic
│   └── utils/
│       ├── auth-middleware.ts     # Existing (no changes)
│       └── avatar-generator.ts    # NEW: First-letter avatar generation
│
frontend/ (Docusaurus)
├── src/
│   ├── components/
│   │   ├── Avatar/
│   │   │   ├── Avatar.tsx         # NEW: Avatar display component
│   │   │   ├── AvatarDropdown.tsx # NEW: Dropdown menu component
│   │   │   └── FirstLetterAvatar.tsx # NEW: Letter-based avatar
│   │   └── Navigation/
│   │       └── AuthNav.tsx        # NEW: Auth-aware navigation
│   ├── hooks/
│   │   └── useAuth.ts             # NEW: Auth state hook
│   └── pages/
│       ├── signup.tsx             # UPDATE: Add avatar + personalization questions
│       └── signin.tsx             # UPDATE: Improve error handling
└── static/
    └── avatars/                   # NEW: 10 predefined avatar images
        ├── avatar-1.png
        ├── avatar-2.png
        └── ... (10 total)
```

**Structure Decision**: This is a web application with separate frontend (Docusaurus) and backend (Auth service). The Auth service is already established in the `Auth/` directory. Frontend components will be added to the Docusaurus structure. The feature extends existing infrastructure without creating new projects.

### Routes

**Frontend Routes** (Docusaurus pages):
- `/profile` - User profile page with two sections:
  - Profile section: Displays user info, avatar, and personalization preferences (read-only view)
  - Settings section: Allows editing avatar selection and personalization preferences
- `/dashboard` - User dashboard (existing from feature 015, redirect destination after sign-up/sign-in)
- `/signup` - Sign-up page (existing, enhanced with avatar selection and personalization questions)
- `/signin` - Sign-in page (existing, enhanced with improved error handling)

**Note**: Profile and Settings are on the same route (`/profile`) with the settings section displayed below the profile content. Clicking "Settings" from the avatar dropdown navigates to `/profile` and shows/displays the settings section.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations - all gates passed.
