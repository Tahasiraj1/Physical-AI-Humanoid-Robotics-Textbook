# API Contract: Chatbot User Context

**Feature**: 017-chatbot-user-context  
**Date**: 2025-01-27  
**Base URL**: Same as existing chatbot API (production: `https://tahasiraj1-humanoid-robotics-chatbot.hf.space`, development: `http://localhost:8000`)

## Overview

This document defines the API contract for the optional `user_context` field in the chatbot API request. The contract extends the existing `/api/chat` endpoint without breaking backward compatibility.

## Endpoint

### POST /api/chat

Send a chat message to the AI agent with optional user personalization context.

**Request Headers**:
```
Content-Type: application/json
```

**Request Body** (TypeScript interface):
```typescript
interface ChatRequest {
  message: string;                    // User message (1-2000 characters) - REQUIRED
  session_id: string;                 // Session identifier (UUID) - REQUIRED
  thread_id?: string;                 // Optional thread ID
  user_id?: string;                   // Optional user ID for authenticated users
  user_context?: {                    // NEW: Optional user personalization context
    progress?: {
      completedModules?: string[];
      completedSections?: Array<{ moduleId: string; sectionId: string }>;
      progressSummary?: Record<string, { progressPercentage: number }>;
    };
    bookmarks?: Array<{
      moduleId: string;
      sectionId: string;
      title?: string;
    }>;
    notes?: Array<{
      moduleId: string;
      sectionId: string;
      content: string;                // Max 200 characters
    }>;
  };
}
```

**Request Example** (with user_context):
```json
{
  "message": "What should I review next?",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "user_id": "user-123",
  "user_context": {
    "progress": {
      "completedModules": ["module-1-ros2-nervous-system", "module-2-perception"],
      "completedSections": [
        { "moduleId": "module-1-ros2-nervous-system", "sectionId": "introduction" },
        { "moduleId": "module-1-ros2-nervous-system", "sectionId": "ros2-basics" }
      ],
      "progressSummary": {
        "module-1-ros2-nervous-system": { "progressPercentage": 71 },
        "module-2-perception": { "progressPercentage": 45 }
      }
    },
    "bookmarks": [
      {
        "moduleId": "module-1-ros2-nervous-system",
        "sectionId": "advanced-topics",
        "title": "Advanced ROS 2 Concepts"
      }
    ],
    "notes": [
      {
        "moduleId": "module-1-ros2-nervous-system",
        "sectionId": "ros2-basics",
        "content": "Key takeaway: ROS 2 uses DDS for communication..."
      }
    ]
  }
}
```

**Request Example** (without user_context - backward compatible):
```json
{
  "message": "What is ROS 2?",
  "session_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

**Response** (Success - 200 OK):
```typescript
interface ChatResponse {
  response: string;                    // AI-generated response text
  citations: Citation[];               // Array of source citations
  session_id: string;                  // Session ID (echoed back)
  thread_id?: string;                  // Optional thread ID
  metadata: {
    model_used: string;                // e.g., "gemini-2.5-flash"
    generation_time: number;           // Response time in seconds
  };
}
```

**Response Example**:
```json
{
  "response": "Based on your progress, you've completed Module 1 and Module 2. I recommend reviewing the bookmarked section on Advanced ROS 2 Concepts, and building on your notes about ROS 2 basics...",
  "citations": [
    {
      "id": "cite-1",
      "text": "Module 1, Section 2.3",
      "url": "/module-1-ros2-nervous-system/advanced-topics"
    }
  ],
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "metadata": {
    "model_used": "gemini-2.5-flash",
    "generation_time": 2.3
  }
}
```

**Error Responses**:

- **400 Bad Request**: Invalid request format or validation error
  ```json
  {
    "error": {
      "type": "ValidationError",
      "message": "Message cannot be empty"
    },
    "timestamp": "2025-01-27T00:00:00Z"
  }
  ```

- **500 Internal Server Error**: Backend processing error
  ```json
  {
    "error": {
      "type": "InternalError",
      "message": "Failed to process request"
    },
    "timestamp": "2025-01-27T00:00:00Z"
  }
  ```

## Field Specifications

### user_context (Optional)

**Type**: Object (optional)

**Description**: User personalization data including progress, bookmarks, and notes. Only sent on the first message of a new session.

**Structure**:
- `progress` (object, optional): User's learning progress
  - `completedModules` (array of strings, optional): Module IDs user has completed
  - `completedSections` (array of objects, optional): Completed sections with moduleId and sectionId
  - `progressSummary` (object, optional): Progress percentages per module
- `bookmarks` (array of objects, optional): User's bookmarked sections
  - Each object: `{ moduleId: string, sectionId: string, title?: string }`
- `notes` (array of objects, optional): User's notes on sections
  - Each object: `{ moduleId: string, sectionId: string, content: string }`
  - `content` must be ≤ 200 characters (enforced by frontend)

**Validation Rules**:
- All fields are optional
- Empty arrays/objects are valid
- Note content must be ≤ 200 characters
- Backend accepts user_context even if empty or partially populated

**When to Include**:
- Only on the first message of a new session
- Only for authenticated users (user_id must be present)
- Frontend responsibility to track and include appropriately

**When to Omit**:
- Subsequent messages in the same session
- Unauthenticated users
- If context fetching fails (graceful degradation)

## Backward Compatibility

- **Existing requests without user_context continue to work**: The field is optional
- **Backend processes requests with or without user_context**: No breaking changes
- **Frontend can send user_context when available**: Enhances responses without breaking unauthenticated users

## Error Handling

- **Missing user_context**: Not an error - backend processes normally
- **Empty user_context**: Not an error - backend processes normally
- **Invalid user_context structure**: Backend may log warning but processes request (graceful degradation)
- **Context fetch failures in frontend**: Frontend sends request without user_context, no error to user

## Performance Considerations

- **Request size**: user_context adds ~1-5KB to request payload (acceptable)
- **Processing time**: Backend processing adds <100ms for context incorporation (acceptable)
- **Frequency**: user_context sent only once per session (minimal overhead)

## Security Considerations

- **user_context contains user-specific data**: Only sent for authenticated users
- **No sensitive data**: Progress, bookmarks, and notes are non-sensitive learning data
- **HTTPS required**: All requests must use HTTPS in production
- **No PII in user_context**: Only module/section IDs and learning data

