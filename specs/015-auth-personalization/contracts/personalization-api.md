# API Contract: Personalization Endpoints

**Feature**: 015-auth-personalization  
**Date**: 2025-01-27  
**Base URL**: `https://auth-service.vercel.app/api/personalization`

## Overview

Custom API endpoints for personalization features (progress tracking, bookmarks, notes, comments, downloads, recommendations).

## Authentication

All endpoints require authentication via Better Auth session cookie. Unauthenticated requests return 401.

**Request Headers**:
```
Cookie: better-auth.session_token=<session-token>
```

## Progress Tracking

### Record Section View

**Endpoint**: `POST /api/personalization/progress`

**Request Body**:
```json
{
  "moduleId": "module-1-ros2-nervous-system",
  "sectionId": "introduction"
}
```

**Response** (Success - 200):
```json
{
  "progress": {
    "id": "uuid",
    "moduleId": "module-1-ros2-nervous-system",
    "sectionId": "introduction",
    "viewedAt": "2025-01-27T00:00:00Z",
    "lastViewedAt": "2025-01-27T00:00:00Z",
    "viewCount": 1,
    "completed": false
  }
}
```

### Get User Progress

**Endpoint**: `GET /api/personalization/progress`

**Query Parameters**:
- `moduleId` (optional) - Filter by module
- `sectionId` (optional) - Filter by section

**Response** (Success - 200):
```json
{
  "progress": [
    {
      "id": "uuid",
      "moduleId": "module-1-ros2-nervous-system",
      "sectionId": "introduction",
      "viewedAt": "2025-01-27T00:00:00Z",
      "lastViewedAt": "2025-01-27T00:00:00Z",
      "viewCount": 3,
      "completed": false
    }
  ],
  "summary": {
    "module-1-ros2-nervous-system": {
      "totalSections": 7,
      "viewedSections": 5,
      "completedSections": 2,
      "progressPercentage": 71
    }
  }
}
```

### Mark Section as Completed

**Endpoint**: `PATCH /api/personalization/progress/:id/complete`

**Request Body**:
```json
{
  "completed": true
}
```

**Response** (Success - 200):
```json
{
  "progress": {
    "id": "uuid",
    "completed": true,
    "updatedAt": "2025-01-27T01:00:00Z"
  }
}
```

## Bookmarks

### Create Bookmark

**Endpoint**: `POST /api/personalization/bookmarks`

**Request Body**:
```json
{
  "moduleId": "module-1-ros2-nervous-system",
  "sectionId": "introduction"
}
```

**Response** (Success - 201):
```json
{
  "bookmark": {
    "id": "uuid",
    "moduleId": "module-1-ros2-nervous-system",
    "sectionId": "introduction",
    "createdAt": "2025-01-27T00:00:00Z"
  }
}
```

**Response** (Error - 409):
```json
{
  "error": {
    "message": "Bookmark already exists",
    "code": "BOOKMARK_EXISTS"
  }
}
```

### Get User Bookmarks

**Endpoint**: `GET /api/personalization/bookmarks`

**Query Parameters**:
- `moduleId` (optional) - Filter by module

**Response** (Success - 200):
```json
{
  "bookmarks": [
    {
      "id": "uuid",
      "moduleId": "module-1-ros2-nervous-system",
      "sectionId": "introduction",
      "createdAt": "2025-01-27T00:00:00Z"
    }
  ]
}
```

### Delete Bookmark

**Endpoint**: `DELETE /api/personalization/bookmarks/:id`

**Response** (Success - 200):
```json
{
  "message": "Bookmark deleted successfully"
}
```

### Check Bookmark Status

**Endpoint**: `GET /api/personalization/bookmarks/check`

**Query Parameters**:
- `moduleId` (required)
- `sectionId` (required)

**Response** (Success - 200):
```json
{
  "isBookmarked": true,
  "bookmarkId": "uuid"
}
```

## Notes

### Create or Update Note

**Endpoint**: `POST /api/personalization/notes`

**Request Body**:
```json
{
  "moduleId": "module-1-ros2-nervous-system",
  "sectionId": "introduction",
  "content": "This is my personal note about this section."
}
```

**Response** (Success - 200/201):
```json
{
  "note": {
    "id": "uuid",
    "moduleId": "module-1-ros2-nervous-system",
    "sectionId": "introduction",
    "content": "This is my personal note about this section.",
    "createdAt": "2025-01-27T00:00:00Z",
    "updatedAt": "2025-01-27T00:00:00Z"
  }
}
```

### Get User Notes

**Endpoint**: `GET /api/personalization/notes`

**Query Parameters**:
- `moduleId` (optional) - Filter by module
- `sectionId` (optional) - Filter by section

**Response** (Success - 200):
```json
{
  "notes": [
    {
      "id": "uuid",
      "moduleId": "module-1-ros2-nervous-system",
      "sectionId": "introduction",
      "content": "This is my personal note.",
      "createdAt": "2025-01-27T00:00:00Z",
      "updatedAt": "2025-01-27T00:00:00Z"
    }
  ]
}
```

### Delete Note

**Endpoint**: `DELETE /api/personalization/notes/:id`

**Response** (Success - 200):
```json
{
  "message": "Note deleted successfully"
}
```

## Comments

### Create Comment

**Endpoint**: `POST /api/personalization/comments`

**Request Body**:
```json
{
  "moduleId": "module-1-ros2-nervous-system",
  "sectionId": "introduction",
  "content": "This is a great section!",
  "parentCommentId": null
}
```

**Response** (Success - 201):
```json
{
  "comment": {
    "id": "uuid",
    "moduleId": "module-1-ros2-nervous-system",
    "sectionId": "introduction",
    "content": "This is a great section!",
    "parentCommentId": null,
    "moderationStatus": "pending",
    "createdAt": "2025-01-27T00:00:00Z",
    "user": {
      "id": "uuid",
      "name": "John Doe",
      "email": "user@example.com"
    }
  }
}
```

### Get Comments for Section

**Endpoint**: `GET /api/personalization/comments`

**Query Parameters**:
- `moduleId` (required)
- `sectionId` (required)
- `parentCommentId` (optional) - Get replies to a comment

**Response** (Success - 200):
```json
{
  "comments": [
    {
      "id": "uuid",
      "moduleId": "module-1-ros2-nervous-system",
      "sectionId": "introduction",
      "content": "This is a great section!",
      "parentCommentId": null,
      "moderationStatus": "approved",
      "createdAt": "2025-01-27T00:00:00Z",
      "user": {
        "id": "uuid",
        "name": "John Doe"
      },
      "replies": [
        {
          "id": "uuid-2",
          "content": "I agree!",
          "parentCommentId": "uuid",
          "createdAt": "2025-01-27T01:00:00Z",
          "user": {
            "id": "uuid-2",
            "name": "Jane Smith"
          }
        }
      ]
    }
  ]
}
```

**Note**: Only approved comments are returned. Unauthenticated users can view comments.

### Flag Comment

**Endpoint**: `POST /api/personalization/comments/:id/flag`

**Response** (Success - 200):
```json
{
  "message": "Comment flagged successfully"
}
```

## Downloads

### Get Downloadable Resources

**Endpoint**: `GET /api/personalization/resources`

**Query Parameters**:
- `moduleId` (optional) - Filter by module

**Response** (Success - 200):
```json
{
  "resources": [
    {
      "id": "uuid",
      "fileName": "module-1-code-examples.zip",
      "fileType": "application/zip",
      "fileSize": 1024000,
      "moduleId": "module-1-ros2-nervous-system",
      "description": "Code examples for Module 1",
      "downloadCount": 42
    }
  ]
}
```

**Note**: This endpoint is accessible to authenticated users only.

### Get Download URL

**Endpoint**: `POST /api/personalization/resources/:id/download`

**Response** (Success - 200):
```json
{
  "downloadUrl": "https://storage.example.com/signed-url?token=...",
  "expiresAt": "2025-01-27T01:00:00Z"
}
```

**Response** (Error - 403):
```json
{
  "error": {
    "message": "Authentication required",
    "code": "UNAUTHORIZED"
  }
}
```

### Get Download History

**Endpoint**: `GET /api/personalization/downloads`

**Response** (Success - 200):
```json
{
  "downloads": [
    {
      "id": "uuid",
      "resourceId": "uuid",
      "resource": {
        "fileName": "module-1-code-examples.zip",
        "moduleId": "module-1-ros2-nervous-system"
      },
      "downloadedAt": "2025-01-27T00:00:00Z"
    }
  ]
}
```

## Recommendations

### Get Personalized Recommendations

**Endpoint**: `GET /api/personalization/recommendations`

**Response** (Success - 200):
```json
{
  "recommendations": [
    {
      "moduleId": "module-2-digital-twins-simulation",
      "reason": "next_in_sequence",
      "priorityScore": 100,
      "description": "Continue your learning journey with Module 2"
    }
  ]
}
```

## User Profile

### Get User Profile

**Endpoint**: `GET /api/personalization/profile`

**Response** (Success - 200):
```json
{
  "profile": {
    "userId": "uuid",
    "learningLevel": "beginner",
    "preferences": {
      "theme": "dark",
      "notifications": true
    }
  }
}
```

### Update User Profile

**Endpoint**: `PATCH /api/personalization/profile`

**Request Body**:
```json
{
  "learningLevel": "intermediate",
  "preferences": {
    "theme": "light"
  }
}
```

**Response** (Success - 200):
```json
{
  "profile": {
    "userId": "uuid",
    "learningLevel": "intermediate",
    "preferences": {
      "theme": "light",
      "notifications": true
    },
    "updatedAt": "2025-01-27T01:00:00Z"
  }
}
```

## Dashboard

### Get Dashboard Data

**Endpoint**: `GET /api/personalization/dashboard`

**Response** (Success - 200):
```json
{
  "progress": {
    "module-1-ros2-nervous-system": {
      "totalSections": 7,
      "viewedSections": 5,
      "completedSections": 2,
      "progressPercentage": 71
    }
  },
  "bookmarks": [
    {
      "id": "uuid",
      "moduleId": "module-1-ros2-nervous-system",
      "sectionId": "introduction"
    }
  ],
  "notes": [
    {
      "id": "uuid",
      "moduleId": "module-1-ros2-nervous-system",
      "sectionId": "introduction",
      "content": "My note..."
    }
  ],
  "recommendations": [
    {
      "moduleId": "module-2-digital-twins-simulation",
      "reason": "next_in_sequence"
    }
  ]
}
```

## Error Responses

All endpoints follow the same error format:

```json
{
  "error": {
    "message": "Human-readable error message",
    "code": "ERROR_CODE"
  }
}
```

## Common Error Codes

- `UNAUTHORIZED` - User is not authenticated
- `NOT_FOUND` - Resource not found
- `VALIDATION_ERROR` - Request validation failed
- `BOOKMARK_EXISTS` - Bookmark already exists
- `RATE_LIMIT_EXCEEDED` - Too many requests

## Rate Limiting

- Progress tracking: 10 requests per minute per user
- Comments: 5 requests per minute per user
- Downloads: 3 requests per minute per user
- Other endpoints: 20 requests per minute per user

