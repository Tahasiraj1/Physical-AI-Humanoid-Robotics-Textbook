# Data Model: User Authentication and Content Personalization

**Feature**: 015-auth-personalization  
**Date**: 2025-01-27

## Overview

This document defines the database schema for user authentication and personalization features. The schema extends Better Auth's default tables with custom tables for personalization data.

## Database: Neon Serverless Postgres

**Connection**: PostgreSQL via Better Auth adapter  
**Schema**: `public` (default)  
**Migration Tool**: Better Auth CLI for auth tables, Drizzle ORM or raw SQL for custom tables

## Better Auth Default Tables

Better Auth automatically creates these tables via `npx @better-auth/cli migrate`:

### `user`
User account information managed by Better Auth.

**Columns**:
- `id` (UUID, PRIMARY KEY) - User identifier
- `name` (VARCHAR) - User's display name
- `email` (VARCHAR, UNIQUE) - User's email address
- `emailVerified` (BOOLEAN) - Email verification status
- `image` (VARCHAR, nullable) - Profile image URL
- `createdAt` (TIMESTAMP) - Account creation timestamp
- `updatedAt` (TIMESTAMP) - Last update timestamp

**Relationships**:
- One-to-many with `session`
- One-to-many with `account` (OAuth, if enabled)
- One-to-many with custom personalization tables

### `session`
User authentication sessions.

**Columns**:
- `id` (VARCHAR, PRIMARY KEY) - Session identifier
- `userId` (UUID, FOREIGN KEY → user.id) - User reference
- `expiresAt` (TIMESTAMP) - Session expiration
- `token` (VARCHAR) - Session token
- `ipAddress` (VARCHAR, nullable) - Client IP
- `userAgent` (VARCHAR, nullable) - Client user agent
- `createdAt` (TIMESTAMP) - Session creation
- `updatedAt` (TIMESTAMP) - Last activity

**Indexes**:
- `idx_session_userId` on `userId`
- `idx_session_expiresAt` on `expiresAt`

### `account`
OAuth provider accounts (optional, for future social auth).

**Columns**:
- `id` (VARCHAR, PRIMARY KEY)
- `userId` (UUID, FOREIGN KEY → user.id)
- `providerId` (VARCHAR) - OAuth provider (e.g., "google")
- `providerAccountId` (VARCHAR) - Provider's user ID
- `accessToken` (VARCHAR, nullable)
- `refreshToken` (VARCHAR, nullable)
- `expiresAt` (TIMESTAMP, nullable)
- `createdAt` (TIMESTAMP)
- `updatedAt` (TIMESTAMP)

### `verification`
Email verification and password reset tokens.

**Columns**:
- `id` (VARCHAR, PRIMARY KEY)
- `identifier` (VARCHAR) - Email address
- `value` (VARCHAR) - Verification token
- `expiresAt` (TIMESTAMP)
- `createdAt` (TIMESTAMP)

### `password`
Password hashes (if email/password auth enabled).

**Columns**:
- `userId` (UUID, PRIMARY KEY, FOREIGN KEY → user.id)
- `hash` (VARCHAR) - Hashed password
- `createdAt` (TIMESTAMP)
- `updatedAt` (TIMESTAMP)

## Custom Personalization Tables

### `user_profile`
Extended user profile information.

**Columns**:
- `userId` (UUID, PRIMARY KEY, FOREIGN KEY → user.id ON DELETE CASCADE)
- `learningLevel` (VARCHAR) - User's self-reported learning level (e.g., "beginner", "intermediate", "advanced")
- `preferences` (JSONB, nullable) - User preferences (theme, notifications, etc.)
- `createdAt` (TIMESTAMP)
- `updatedAt` (TIMESTAMP)

**Constraints**:
- One profile per user (1:1 relationship)

### `reading_progress`
Tracks user's progress through module sections.

**Columns**:
- `id` (UUID, PRIMARY KEY, DEFAULT gen_random_uuid())
- `userId` (UUID, FOREIGN KEY → user.id ON DELETE CASCADE)
- `moduleId` (VARCHAR) - Module identifier (e.g., "module-1-ros2-nervous-system")
- `sectionId` (VARCHAR) - Section identifier (e.g., "introduction")
- `viewedAt` (TIMESTAMP, DEFAULT NOW()) - First view timestamp
- `lastViewedAt` (TIMESTAMP, DEFAULT NOW()) - Last view timestamp
- `viewCount` (INTEGER, DEFAULT 1) - Number of times viewed
- `completed` (BOOLEAN, DEFAULT false) - Marked as completed by user
- `createdAt` (TIMESTAMP, DEFAULT NOW())
- `updatedAt` (TIMESTAMP, DEFAULT NOW())

**Indexes**:
- `idx_reading_progress_userId` on `userId`
- `idx_reading_progress_moduleId` on `moduleId`
- `UNIQUE idx_reading_progress_user_module_section` on (`userId`, `moduleId`, `sectionId`)

**Constraints**:
- Unique combination of userId, moduleId, sectionId (one progress record per user per section)

### `bookmark`
User bookmarks for module sections.

**Columns**:
- `id` (UUID, PRIMARY KEY, DEFAULT gen_random_uuid())
- `userId` (UUID, FOREIGN KEY → user.id ON DELETE CASCADE)
- `moduleId` (VARCHAR) - Module identifier
- `sectionId` (VARCHAR) - Section identifier
- `createdAt` (TIMESTAMP, DEFAULT NOW())

**Indexes**:
- `idx_bookmark_userId` on `userId`
- `idx_bookmark_moduleId` on `moduleId`
- `UNIQUE idx_bookmark_user_module_section` on (`userId`, `moduleId`, `sectionId`)

**Constraints**:
- Unique combination of userId, moduleId, sectionId (prevent duplicate bookmarks)

### `user_note`
Personal notes on module sections.

**Columns**:
- `id` (UUID, PRIMARY KEY, DEFAULT gen_random_uuid())
- `userId` (UUID, FOREIGN KEY → user.id ON DELETE CASCADE)
- `moduleId` (VARCHAR) - Module identifier
- `sectionId` (VARCHAR) - Section identifier
- `content` (TEXT) - Note content
- `createdAt` (TIMESTAMP, DEFAULT NOW())
- `updatedAt` (TIMESTAMP, DEFAULT NOW())

**Indexes**:
- `idx_user_note_userId` on `userId`
- `idx_user_note_moduleId` on `moduleId`
- `idx_user_note_sectionId` on `sectionId`

**Constraints**:
- One note per user per section (can be updated, not duplicated)

### `user_comment`
Comments and discussions on module sections.

**Columns**:
- `id` (UUID, PRIMARY KEY, DEFAULT gen_random_uuid())
- `userId` (UUID, FOREIGN KEY → user.id ON DELETE CASCADE)
- `moduleId` (VARCHAR) - Module identifier
- `sectionId` (VARCHAR) - Section identifier
- `parentCommentId` (UUID, nullable, FOREIGN KEY → user_comment.id ON DELETE CASCADE) - For reply comments
- `content` (TEXT) - Comment content
- `moderationStatus` (VARCHAR, DEFAULT 'approved') - 'pending', 'approved', 'rejected', 'flagged'
- `flaggedCount` (INTEGER, DEFAULT 0) - Number of times flagged by users
- `createdAt` (TIMESTAMP, DEFAULT NOW())
- `updatedAt` (TIMESTAMP, DEFAULT NOW())

**Indexes**:
- `idx_user_comment_userId` on `userId`
- `idx_user_comment_moduleId` on `moduleId`
- `idx_user_comment_sectionId` on `sectionId`
- `idx_user_comment_parentCommentId` on `parentCommentId`
- `idx_user_comment_createdAt` on `createdAt` (for chronological ordering)

**Constraints**:
- Self-referential foreign key for reply comments

### `chat_session`
Chatbot conversation sessions linked to user accounts.

**Columns**:
- `id` (UUID, PRIMARY KEY, DEFAULT gen_random_uuid())
- `userId` (UUID, FOREIGN KEY → user.id ON DELETE CASCADE)
- `sessionId` (VARCHAR) - Chatbot session identifier (from existing FastAPI backend)
- `messages` (JSONB) - Array of chat messages
- `startedAt` (TIMESTAMP, DEFAULT NOW())
- `lastActivityAt` (TIMESTAMP, DEFAULT NOW())

**Indexes**:
- `idx_chat_session_userId` on `userId`
- `idx_chat_session_sessionId` on `sessionId`

**Note**: This table links existing chatbot sessions (identified by `sessionId`) to user accounts. The FastAPI backend continues to manage session state; this table provides the user linkage.

### `downloadable_resource`
Metadata for downloadable resources (files stored in cloud storage).

**Columns**:
- `id` (UUID, PRIMARY KEY, DEFAULT gen_random_uuid())
- `fileName` (VARCHAR) - Original file name
- `fileType` (VARCHAR) - MIME type (e.g., "application/pdf")
- `fileSize` (BIGINT) - File size in bytes
- `storageUrl` (VARCHAR) - Cloud storage URL/path
- `moduleId` (VARCHAR, nullable) - Associated module (if applicable)
- `description` (TEXT, nullable) - Resource description
- `downloadCount` (INTEGER, DEFAULT 0) - Total download count
- `createdAt` (TIMESTAMP, DEFAULT NOW())
- `updatedAt` (TIMESTAMP, DEFAULT NOW())

**Indexes**:
- `idx_downloadable_resource_moduleId` on `moduleId`

### `download_history`
Records of user downloads.

**Columns**:
- `id` (UUID, PRIMARY KEY, DEFAULT gen_random_uuid())
- `userId` (UUID, FOREIGN KEY → user.id ON DELETE CASCADE)
- `resourceId` (UUID, FOREIGN KEY → downloadable_resource.id ON DELETE CASCADE)
- `downloadedAt` (TIMESTAMP, DEFAULT NOW())

**Indexes**:
- `idx_download_history_userId` on `userId`
- `idx_download_history_resourceId` on `resourceId`
- `idx_download_history_downloadedAt` on `downloadedAt`

### `module_recommendation`
Generated module recommendations (optional, can be computed on-the-fly).

**Columns**:
- `id` (UUID, PRIMARY KEY, DEFAULT gen_random_uuid())
- `userId` (UUID, FOREIGN KEY → user.id ON DELETE CASCADE)
- `recommendedModuleId` (VARCHAR) - Recommended module identifier
- `reason` (VARCHAR) - Recommendation reason (e.g., "next_in_sequence", "based_on_progress")
- `priorityScore` (INTEGER) - Recommendation priority (higher = more relevant)
- `createdAt` (TIMESTAMP, DEFAULT NOW())
- `expiresAt` (TIMESTAMP, nullable) - When recommendation expires (if applicable)

**Indexes**:
- `idx_module_recommendation_userId` on `userId`
- `idx_module_recommendation_priorityScore` on `priorityScore` DESC

**Note**: Recommendations can be computed on-the-fly from progress data, but caching in this table can improve performance.

## Entity Relationships

```
user (Better Auth)
├── 1:1 user_profile
├── 1:N session (Better Auth)
├── 1:N reading_progress
├── 1:N bookmark
├── 1:N user_note
├── 1:N user_comment
├── 1:N chat_session
└── 1:N download_history

user_comment
└── 1:N user_comment (self-referential for replies)

downloadable_resource
└── 1:N download_history
```

## Data Validation Rules

### Reading Progress
- `moduleId` and `sectionId` must match existing Docusaurus content structure
- `viewCount` must be >= 1
- `completed` can only be set to true by user action (not automatic)

### Bookmarks
- Cannot bookmark same section twice (enforced by unique constraint)
- `moduleId` and `sectionId` must reference valid content

### User Notes
- `content` cannot be empty
- Maximum length: 10,000 characters (enforced application-side)

### User Comments
- `content` cannot be empty
- Maximum length: 5,000 characters (enforced application-side)
- `moderationStatus` must be one of: 'pending', 'approved', 'rejected', 'flagged'

### Downloadable Resources
- `fileSize` must be > 0
- `storageUrl` must be valid URL or storage path

## Migration Strategy

1. **Better Auth Tables**: Run `npx @better-auth/cli migrate` to create auth tables
2. **Custom Tables**: Create custom tables using Drizzle ORM migrations or raw SQL
3. **Indexes**: Create indexes after table creation for performance
4. **Foreign Keys**: Add foreign key constraints with ON DELETE CASCADE for data integrity

## Performance Considerations

- Indexes on frequently queried columns (userId, moduleId, sectionId)
- JSONB columns for flexible data (messages, preferences) with GIN indexes if needed
- Partitioning not required initially (Neon handles scaling)
- Consider materialized views for progress percentages if query performance becomes an issue

## Data Retention

- User data deleted on account deletion (CASCADE)
- Comments can be anonymized instead of deleted (set userId to NULL, mark as deleted)
- Download history retained for analytics (can be anonymized)
- Chat sessions retained per user (can be purged after inactivity period)

