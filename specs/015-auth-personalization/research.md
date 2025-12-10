# Research: User Authentication and Content Personalization

**Feature**: 015-auth-personalization  
**Date**: 2025-01-27  
**Status**: Complete

## Research Summary

This document consolidates technical research and decisions for implementing Better Auth authentication and content personalization features in the Physical AI Humanoid Robotics Textbook.

## 1. Better Auth Integration with React/Docusaurus

### Decision
Use `better-auth/react` package for client-side authentication in Docusaurus React components.

### Rationale
- Better Auth provides React-specific hooks (`useSession`, `useUser`) that integrate seamlessly with React components
- Docusaurus uses React, so Better Auth React client is the natural fit
- Reactive state management using nano-store ensures UI updates automatically on auth state changes
- No need for manual state synchronization

### Alternatives Considered
- Vanilla `better-auth/client`: Would require manual state management and React integration
- Custom auth solution: Too much development overhead for standard authentication needs

### Implementation Pattern
```typescript
// src/lib/auth-client.ts
import { createAuthClient } from "better-auth/react"

export const authClient = createAuthClient({
  baseURL: process.env.NEXT_PUBLIC_AUTH_URL || "http://localhost:3000"
})
```

## 2. Better Auth Express.js Server Setup

### Decision
Create separate Express.js server in `Auth/` folder, mount Better Auth handler at `/api/auth/*` route.

### Rationale
- Better Auth requires a backend server to handle authentication requests
- Express.js is well-supported by Better Auth with `toNodeHandler` utility
- Separate service enables independent deployment and scaling
- Matches project requirement for separate Auth folder

### Alternatives Considered
- Next.js API routes: Would require Next.js framework, not suitable for separate service
- Serverless functions only: Better Auth works better with persistent server for session management

### Implementation Pattern
```typescript
// Auth/src/index.ts
import express from "express"
import cors from "cors"
import { toNodeHandler } from "better-auth/node"
import { auth } from "./auth"

const app = express()

app.use(cors({
  origin: process.env.FRONTEND_URL,
  credentials: true
}))

app.all("/api/auth/*", toNodeHandler(auth))
```

## 3. Vercel Deployment for Auth Service

### Decision
Deploy Express.js auth service to Vercel as serverless functions using Vercel's Node.js runtime.

### Rationale
- Vercel supports Express.js applications natively
- Serverless deployment scales automatically
- Easy environment variable management
- Separate deployment from Docusaurus (GitHub Pages)

### Alternatives Considered
- Railway/Render: Additional service to manage, Vercel is simpler for serverless
- Self-hosted: Too much infrastructure overhead

### Configuration
- Use `vercel.json` for routing configuration
- Environment variables configured in Vercel dashboard
- Base URL will be Vercel deployment URL

## 4. Neon Postgres Database Schema

### Decision
Use Better Auth's built-in PostgreSQL adapter with Neon Serverless Postgres. Extend schema with custom tables for personalization data.

### Rationale
- Better Auth provides PostgreSQL adapter with automatic schema generation
- Neon Serverless Postgres is compatible with Better Auth's requirements
- Serverless database scales automatically
- Can use Better Auth CLI for migrations

### Schema Structure
**Better Auth Default Tables** (auto-generated):
- `user` - User accounts
- `session` - User sessions
- `account` - OAuth accounts (if added later)
- `verification` - Email verification tokens
- `password` - Password hashes

**Custom Tables** (manual creation):
- `reading_progress` - Module/section progress tracking
- `bookmark` - User bookmarks
- `user_note` - Personal notes
- `user_comment` - Section comments
- `chat_session` - Chatbot session history
- `download_history` - Resource download tracking
- `module_recommendation` - Generated recommendations (optional, can be computed)

### Migration Strategy
1. Use Better Auth CLI: `npx @better-auth/cli migrate` for auth tables
2. Create custom tables manually or with migration tool
3. Use Drizzle ORM or raw SQL for custom tables

## 5. CORS Configuration

### Decision
Configure CORS middleware in Express server to allow requests from Docusaurus domain with credentials enabled.

### Rationale
- Docusaurus (GitHub Pages) and Auth service (Vercel) are on different domains
- CORS is required for cross-origin requests
- Credentials must be enabled for cookie-based session management

### Configuration
```typescript
app.use(cors({
  origin: process.env.FRONTEND_URL, // Docusaurus site URL
  methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
  credentials: true,
  allowedHeaders: ["Content-Type", "Authorization"]
}))
```

### Environment Variables
- `FRONTEND_URL`: Docusaurus site URL (e.g., `https://tahasiraj1.github.io`)
- `AUTH_URL`: Auth service URL (Vercel deployment URL)

## 6. Progress Tracking Implementation

### Decision
Track progress client-side on page load, send to backend API endpoint. Use page view detection via React useEffect hooks.

### Rationale
- Page view detection is straightforward in React
- Client-side tracking reduces server load
- Can batch progress updates
- Works with Docusaurus's React-based architecture

### Implementation Pattern
```typescript
// Component on each module section page
useEffect(() => {
  if (user) {
    trackProgress(moduleId, sectionId)
  }
}, [moduleId, sectionId, user])
```

### Backend Endpoint
- `POST /api/personalization/progress` - Record section view
- `GET /api/personalization/progress` - Get user progress

## 7. Chatbot Session Linking

### Decision
Extend existing FastAPI chatbot backend to accept user ID in requests. Store chat sessions in database linked to user accounts.

### Rationale
- Existing chatbot is FastAPI-based, separate from auth service
- Can add user context to existing API without major refactoring
- Chat history stored in Postgres alongside other user data

### Integration Pattern
1. Frontend sends user ID (from Better Auth session) with chat requests
2. FastAPI backend stores messages with user_id
3. Chat history retrieved by user_id

### Database Table
```sql
CREATE TABLE chat_session (
  id UUID PRIMARY KEY,
  user_id UUID REFERENCES "user"(id),
  session_id VARCHAR(255),
  messages JSONB,
  created_at TIMESTAMP,
  updated_at TIMESTAMP
)
```

## 8. File Storage for Downloadable Resources

### Decision
Use cloud storage service (AWS S3, Cloudflare R2, or Vercel Blob) for downloadable resources. Auth service generates signed URLs for authenticated users.

### Rationale
- Static files shouldn't be in database
- Cloud storage provides CDN and scalability
- Signed URLs provide secure, time-limited access
- Can integrate with Vercel Blob for simplicity

### Alternatives Considered
- Database storage: Not suitable for large files
- GitHub releases: Less flexible, harder to manage access

### Implementation
- Store file metadata in database
- Generate signed URLs via auth service endpoint
- Frontend downloads from signed URL

## 9. Comment Moderation

### Decision
Implement basic moderation: manual review initially, with flagging system. Store moderation status in database.

### Rationale
- Start simple with manual moderation
- Can add automated moderation later
- Flagging system allows community participation

### Database Schema
```sql
ALTER TABLE user_comment ADD COLUMN moderation_status VARCHAR(20) DEFAULT 'pending';
ALTER TABLE user_comment ADD COLUMN flagged_count INTEGER DEFAULT 0;
```

## 10. Environment Variables

### Auth Service (.env)
```
DATABASE_URL=postgresql://... (Neon connection string)
BETTER_AUTH_SECRET=... (random secret for Better Auth)
BETTER_AUTH_URL=https://auth-service.vercel.app
FRONTEND_URL=https://tahasiraj1.github.io/Physical-AI-Humanoid-Robotics-Textbook
EMAIL_SERVER=... (for password reset)
```

### Docusaurus Frontend
```
NEXT_PUBLIC_AUTH_URL=https://auth-service.vercel.app
```

## 11. Testing Strategy

### Decision
- Unit tests: Jest/Vitest for auth client and components
- Integration tests: Test auth flows end-to-end
- E2E tests: Playwright for user journeys

### Rationale
- Comprehensive testing ensures reliability
- E2E tests validate user experience
- Integration tests catch API issues

## 12. Security Considerations

### Decisions
1. **Password Requirements**: Minimum 8 characters, require complexity (enforced by Better Auth)
2. **Session Management**: HTTP-only cookies, secure flag in production
3. **CORS**: Whitelist specific frontend domain only
4. **Rate Limiting**: Implement on auth endpoints (Vercel provides basic rate limiting)
5. **SQL Injection**: Use parameterized queries (Better Auth handles this)

### Rationale
- Better Auth provides secure defaults
- Additional security measures protect against common attacks
- Rate limiting prevents abuse

## 13. Error Handling

### Decision
- Client-side: User-friendly error messages
- Server-side: Log errors, return generic messages to client
- Network errors: Retry with exponential backoff

### Rationale
- Good UX requires clear error messages
- Security: Don't expose internal errors to clients
- Resilience: Handle network failures gracefully

## Summary of Key Decisions

| Area | Decision | Rationale |
|------|----------|-----------|
| Client Library | `better-auth/react` | Native React integration |
| Backend Framework | Express.js | Better Auth support, separate service |
| Deployment | Vercel (serverless) | Easy scaling, separate from Docusaurus |
| Database | Neon Postgres | Serverless, Better Auth compatible |
| CORS | Express cors middleware | Required for cross-origin requests |
| Progress Tracking | Client-side + API | Efficient, works with React |
| Chatbot Integration | Extend FastAPI | Minimal changes to existing system |
| File Storage | Cloud storage + signed URLs | Scalable, secure access |
| Moderation | Manual + flagging | Start simple, expand later |

## Next Steps

1. Set up Auth folder structure
2. Configure Better Auth with PostgreSQL adapter
3. Create database schema (Better Auth + custom tables)
4. Implement Express server with CORS
5. Create React auth client in Docusaurus
6. Build personalization components
7. Integrate with existing chatbot
8. Set up Vercel deployment

