# API Contract: Better Auth Endpoints

**Feature**: 015-auth-personalization  
**Date**: 2025-01-27  
**Base URL**: `https://auth-service.vercel.app/api/auth`

## Overview

Better Auth provides built-in authentication endpoints. This document describes the endpoints used by the frontend.

## Authentication Endpoints

### Sign Up (Email/Password)

**Endpoint**: `POST /api/auth/sign-up/email`

**Request Body**:
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123!",
  "name": "John Doe"
}
```

**Response** (Success - 200):
```json
{
  "user": {
    "id": "uuid",
    "email": "user@example.com",
    "name": "John Doe",
    "emailVerified": false,
    "createdAt": "2025-01-27T00:00:00Z"
  },
  "session": {
    "id": "session-id",
    "expiresAt": "2025-01-28T00:00:00Z"
  }
}
```

**Response** (Error - 400):
```json
{
  "error": {
    "message": "Email already exists",
    "code": "EMAIL_ALREADY_EXISTS"
  }
}
```

### Sign In (Email/Password)

**Endpoint**: `POST /api/auth/sign-in/email`

**Request Body**:
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123!"
}
```

**Response** (Success - 200):
```json
{
  "user": {
    "id": "uuid",
    "email": "user@example.com",
    "name": "John Doe"
  },
  "session": {
    "id": "session-id",
    "expiresAt": "2025-01-28T00:00:00Z"
  }
}
```

**Response** (Error - 401):
```json
{
  "error": {
    "message": "Invalid email or password",
    "code": "INVALID_CREDENTIALS"
  }
}
```

### Sign Out

**Endpoint**: `POST /api/auth/sign-out`

**Request Headers**:
```
Cookie: better-auth.session_token=<session-token>
```

**Response** (Success - 200):
```json
{
  "message": "Signed out successfully"
}
```

### Get Session

**Endpoint**: `GET /api/auth/session`

**Request Headers**:
```
Cookie: better-auth.session_token=<session-token>
```

**Response** (Success - 200):
```json
{
  "user": {
    "id": "uuid",
    "email": "user@example.com",
    "name": "John Doe",
    "emailVerified": true
  },
  "session": {
    "id": "session-id",
    "expiresAt": "2025-01-28T00:00:00Z"
  }
}
```

**Response** (No Session - 401):
```json
{
  "user": null,
  "session": null
}
```

### Update User Profile

**Endpoint**: `PATCH /api/auth/user`

**Request Headers**:
```
Cookie: better-auth.session_token=<session-token>
```

**Request Body**:
```json
{
  "name": "John Updated"
}
```

**Response** (Success - 200):
```json
{
  "user": {
    "id": "uuid",
    "email": "user@example.com",
    "name": "John Updated",
    "updatedAt": "2025-01-27T01:00:00Z"
  }
}
```

### Change Password

**Endpoint**: `POST /api/auth/change-password`

**Request Headers**:
```
Cookie: better-auth.session_token=<session-token>
```

**Request Body**:
```json
{
  "currentPassword": "OldPassword123!",
  "newPassword": "NewPassword123!"
}
```

**Response** (Success - 200):
```json
{
  "message": "Password changed successfully"
}
```

### Request Password Reset

**Endpoint**: `POST /api/auth/forget-password`

**Request Body**:
```json
{
  "email": "user@example.com"
}
```

**Response** (Success - 200):
```json
{
  "message": "Password reset email sent"
}
```

### Reset Password

**Endpoint**: `POST /api/auth/reset-password`

**Request Body**:
```json
{
  "token": "reset-token-from-email",
  "newPassword": "NewPassword123!"
}
```

**Response** (Success - 200):
```json
{
  "message": "Password reset successfully"
}
```

## Error Response Format

All error responses follow this format:

```json
{
  "error": {
    "message": "Human-readable error message",
    "code": "ERROR_CODE"
  }
}
```

## Common Error Codes

- `EMAIL_ALREADY_EXISTS` - Email is already registered
- `INVALID_CREDENTIALS` - Email or password is incorrect
- `INVALID_TOKEN` - Token is invalid or expired
- `UNAUTHORIZED` - User is not authenticated
- `VALIDATION_ERROR` - Request validation failed
- `RATE_LIMIT_EXCEEDED` - Too many requests

## Authentication

All authenticated endpoints require a session cookie set by Better Auth. The cookie is automatically managed by the Better Auth client.

## CORS

CORS is configured to allow requests from the Docusaurus frontend domain. Credentials (cookies) are enabled for session management.

