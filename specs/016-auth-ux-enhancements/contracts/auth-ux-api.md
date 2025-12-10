# API Contracts: Enhanced Authentication UX

**Feature**: 016-auth-ux-enhancements  
**Date**: 2025-12-08  
**Base URL**: `http://localhost:3000/api` (development) / `https://auth-service.vercel.app/api` (production)

## Overview

This document defines API contracts for avatar management and user profile updates related to the enhanced authentication UX feature. These endpoints extend the existing Better Auth API and personalization API.

## Authentication

All endpoints (except public avatar listing) require authentication via Better Auth session cookie.

**Authentication Header**: Session cookie set by Better Auth (`better-auth.session_token`)

## Endpoints

### 1. Get Available Avatars

**GET** `/avatar/list`

**Description**: Retrieve all active avatars available for user selection.

**Authentication**: Not required (public endpoint)

**Query Parameters**: None

**Response**:
```json
{
  "avatars": [
    {
      "id": "avatar-1",
      "imageUrl": "/static/avatars/avatar-1.png",
      "displayName": "Robot Avatar 1",
      "isActive": true
    },
    {
      "id": "avatar-2",
      "imageUrl": "/static/avatars/avatar-2.png",
      "displayName": "Robot Avatar 2",
      "isActive": true
    }
    // ... up to 10 avatars
  ]
}
```

**Status Codes**:
- `200 OK`: Success
- `500 Internal Server Error`: Server error

**Error Response**:
```json
{
  "error": "Failed to fetch avatars",
  "message": "Database connection error"
}
```

---

### 2. Get User Avatar Data

**GET** `/personalization/avatar`

**Description**: Get current user's avatar selection and data needed for display.

**Authentication**: Required

**Query Parameters**: None

**Response**:
```json
{
  "avatar": {
    "hasSelectedAvatar": true,
    "selectedAvatarId": "avatar-3",
    "selectedAvatarImageUrl": "/static/avatars/avatar-3.png",
    "userName": "John Doe",
    "avatarDisplayType": "predefined",
    "firstLetter": null
  }
}
```

**Response (No Avatar Selected)**:
```json
{
  "avatar": {
    "hasSelectedAvatar": false,
    "selectedAvatarId": null,
    "selectedAvatarImageUrl": null,
    "userName": "John Doe",
    "avatarDisplayType": "letter",
    "firstLetter": "J"
  }
}
```

**Status Codes**:
- `200 OK`: Success
- `401 Unauthorized`: Not authenticated
- `500 Internal Server Error`: Server error

**Error Response**:
```json
{
  "error": "Failed to fetch avatar data",
  "message": "User not found"
}
```

---

### 3. Update User Avatar Selection

**PUT** `/personalization/avatar`

**Description**: Update user's selected avatar.

**Authentication**: Required

**Request Body**:
```json
{
  "avatarId": "avatar-5"
}
```

**Request Body (Clear Selection)**:
```json
{
  "avatarId": null
}
```

**Response**:
```json
{
  "success": true,
  "avatar": {
    "selectedAvatarId": "avatar-5",
    "selectedAvatarImageUrl": "/static/avatars/avatar-5.png"
  }
}
```

**Status Codes**:
- `200 OK`: Success
- `400 Bad Request`: Invalid avatar ID or avatar not active
- `401 Unauthorized`: Not authenticated
- `404 Not Found`: Avatar not found
- `500 Internal Server Error`: Server error

**Error Response**:
```json
{
  "error": "Invalid avatar selection",
  "message": "Avatar with ID 'avatar-99' not found or inactive"
}
```

**Validation Rules**:
- `avatarId` must be a valid UUID or avatar identifier
- If `avatarId` is provided, avatar must exist and be active
- `avatarId` can be `null` to clear selection

---

### 4. Update User Personalization Preferences

**PUT** `/personalization/preferences`

**Description**: Update user's personalization preferences (software, hardware, programming languages).

**Authentication**: Required

**Request Body**:
```json
{
  "softwarePreferences": ["ROS 2", "Gazebo", "Python"],
  "hardwarePreferences": ["Humanoid robots", "Manipulators"],
  "programmingLanguagePreferences": ["Python", "C++", "JavaScript"]
}
```

**Request Body (Partial Update)**:
```json
{
  "softwarePreferences": ["ROS 2"],
  "hardwarePreferences": null,
  "programmingLanguagePreferences": null
}
```

**Request Body (Clear All)**:
```json
{
  "softwarePreferences": [],
  "hardwarePreferences": [],
  "programmingLanguagePreferences": []
}
```

**Response**:
```json
{
  "success": true,
  "preferences": {
    "softwarePreferences": ["ROS 2", "Gazebo", "Python"],
    "hardwarePreferences": ["Humanoid robots", "Manipulators"],
    "programmingLanguagePreferences": ["Python", "C++", "JavaScript"]
  }
}
```

**Status Codes**:
- `200 OK`: Success
- `400 Bad Request`: Invalid preference data
- `401 Unauthorized`: Not authenticated
- `500 Internal Server Error`: Server error

**Error Response**:
```json
{
  "error": "Failed to update preferences",
  "message": "Invalid preference format"
}
```

**Validation Rules**:
- All preference fields are optional
- Each preference field must be an array of strings or null
- Arrays can be empty `[]`
- Preference values should be validated against predefined option lists (application-level)

---

### 5. Get User Profile (Extended)

**GET** `/personalization/profile`

**Description**: Get complete user profile including avatar and personalization preferences.

**Authentication**: Required

**Query Parameters**: None

**Response**:
```json
{
  "profile": {
    "userId": "user-123",
    "name": "John Doe",
    "email": "john@example.com",
    "learningLevel": "intermediate",
    "selectedAvatarId": "avatar-3",
    "selectedAvatarImageUrl": "/static/avatars/avatar-3.png",
    "softwarePreferences": ["ROS 2", "Gazebo"],
    "hardwarePreferences": ["Humanoid robots"],
    "programmingLanguagePreferences": ["Python", "C++"],
    "createdAt": "2025-12-01T10:00:00Z",
    "updatedAt": "2025-12-08T15:30:00Z"
  }
}
```

**Status Codes**:
- `200 OK`: Success
- `401 Unauthorized`: Not authenticated
- `500 Internal Server Error`: Server error

---

## Better Auth Endpoints (Existing)

The following Better Auth endpoints are used but not modified:

### Sign Up

**POST** `/api/auth/sign-up`

**Description**: Create new user account. Automatically signs user in after successful sign-up.

**Request Body**:
```json
{
  "email": "user@example.com",
  "password": "securepassword",
  "name": "User Name"
}
```

**Response** (Success):
```json
{
  "data": {
    "user": {
      "id": "user-123",
      "name": "User Name",
      "email": "user@example.com"
    },
    "session": {
      "id": "session-456",
      "expiresAt": "2025-12-09T10:00:00Z"
    }
  },
  "error": null
}
```

**Response** (Error - Account Exists):
```json
{
  "data": null,
  "error": {
    "message": "An account with this email already exists",
    "code": "ACCOUNT_EXISTS"
  }
}
```

**Note**: Frontend should NOT redirect on error. Display error message and keep user on sign-up page.

---

### Sign In

**POST** `/api/auth/sign-in`

**Description**: Authenticate user and create session.

**Request Body**:
```json
{
  "email": "user@example.com",
  "password": "securepassword"
}
```

**Response** (Success):
```json
{
  "data": {
    "user": {
      "id": "user-123",
      "name": "User Name",
      "email": "user@example.com"
    },
    "session": {
      "id": "session-456",
      "expiresAt": "2025-12-09T10:00:00Z"
    }
  },
  "error": null
}
```

**Response** (Error - Invalid Credentials):
```json
{
  "data": null,
  "error": {
    "message": "Invalid email or password",
    "code": "INVALID_CREDENTIALS"
  }
}
```

**Note**: Frontend should NOT redirect on error. Display error message and keep user on sign-in page.

---

### Sign Out

**POST** `/api/auth/sign-out`

**Description**: End user session.

**Authentication**: Required

**Response**:
```json
{
  "data": null,
  "error": null
}
```

**Status Codes**:
- `200 OK`: Success
- `401 Unauthorized`: Not authenticated

---

### Get Session

**GET** `/api/auth/get-session`

**Description**: Get current user session and user data.

**Authentication**: Optional (returns null if not authenticated)

**Response** (Authenticated):
```json
{
  "data": {
    "user": {
      "id": "user-123",
      "name": "User Name",
      "email": "user@example.com"
    },
    "session": {
      "id": "session-456",
      "expiresAt": "2025-12-09T10:00:00Z"
    }
  },
  "error": null
}
```

**Response** (Not Authenticated):
```json
{
  "data": null,
  "error": null
}
```

---

## Error Handling

All endpoints follow consistent error response format:

```json
{
  "error": "Error type",
  "message": "Detailed error message"
}
```

**Common Error Codes**:
- `ACCOUNT_EXISTS`: Email already registered (sign-up)
- `INVALID_CREDENTIALS`: Wrong email/password (sign-in)
- `UNAUTHORIZED`: Authentication required
- `NOT_FOUND`: Resource not found
- `VALIDATION_ERROR`: Invalid request data
- `SERVER_ERROR`: Internal server error

---

## Rate Limiting

- Sign-up: 5 attempts per email per hour
- Sign-in: 10 attempts per IP per hour
- Avatar updates: 20 requests per user per minute
- Preference updates: 20 requests per user per minute

---

## CORS Configuration

- Allowed origins: Frontend URL (from environment variable)
- Allowed methods: GET, POST, PUT, DELETE, OPTIONS
- Allowed headers: Content-Type, Authorization, Cookie
- Credentials: true (for session cookies)

---

## Versioning

Current version: `v1` (implicit, no version prefix in URLs)

Future breaking changes will be versioned as `/api/v2/...`

