# Quickstart: Chatbot User Context Development

**Feature**: 017-chatbot-user-context  
**Date**: 2025-01-27  
**Phase**: 1 - Design & Contracts

## Prerequisites

- Node.js >= 20.0 (for frontend)
- Python 3.11+ (for backend)
- Existing Docusaurus development environment
- Existing FastAPI backend running
- Existing personalization APIs accessible (progress, bookmarks, notes)
- User authentication system (AuthProvider) configured

## Overview

This feature adds user personalization context to chatbot requests. The frontend fetches user data (progress, bookmarks, notes) and includes it in the chatbot API request on the first message of each new session. The backend accepts this optional context and uses it to personalize responses.

## Frontend Setup

### 1. Update Type Definitions

**File**: `src/components/ChatWidget/types.ts`

Add `user_context` field to `ChatRequest` interface:

```typescript
export interface UserContext {
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
    content: string; // Max 200 characters
  }>;
}

export interface ChatRequest {
  message: string;
  session_id: string;
  thread_id?: string;
  user_id?: string;
  user_context?: UserContext; // NEW FIELD
}
```

### 2. Create User Context Fetcher

**File**: `src/components/ChatWidget/services/chatService.ts`

Add function to fetch user context:

```typescript
import { progressService } from '@site/src/components/Personalization/services/progressService';
import { bookmarkService } from '@site/src/components/Personalization/services/bookmarkService';
import { noteService } from '@site/src/components/Personalization/services/noteService';
import type { UserContext } from '../types';

/**
 * Fetch user personalization data for chatbot context
 * Returns undefined if user is not authenticated or fetch fails
 */
async function fetchUserContext(userId: string): Promise<UserContext | undefined> {
  try {
    // Fetch all user data in parallel (use allSettled so one failure doesn't block others)
    const [progressData, bookmarksData, notesData] = await Promise.allSettled([
      progressService.getUserProgress(),
      bookmarkService.getBookmarks(),
      noteService.getNotes(),
    ]);

    const context: UserContext = {};

    // Process progress data
    if (progressData.status === 'fulfilled') {
      const progress = progressData.value;
      const completedSections = progress.progress?.filter((p: any) => p.completed) || [];
      const completedModules = new Set(completedSections.map((p: any) => p.moduleId));
      
      context.progress = {
        completedModules: Array.from(completedModules),
        completedSections: completedSections.map((p: any) => ({
          moduleId: p.moduleId,
          sectionId: p.sectionId,
        })),
        progressSummary: progress.summary || {},
      };
    }

    // Process bookmarks
    if (bookmarksData.status === 'fulfilled') {
      context.bookmarks = bookmarksData.value.bookmarks?.map((b: any) => ({
        moduleId: b.moduleId,
        sectionId: b.sectionId,
        title: b.title,
      })) || [];
    }

    // Process notes (limit content length to avoid large payloads)
    if (notesData.status === 'fulfilled') {
      context.notes = notesData.value.notes?.map((n: any) => ({
        moduleId: n.moduleId,
        sectionId: n.sectionId,
        content: n.content.substring(0, 200), // Limit to 200 characters
      })) || [];
    }

    return context;
  } catch (error) {
    console.warn('[chatService] Failed to fetch user context:', error);
    return undefined; // Return undefined if fetch fails - chatbot will work without context
  }
}
```

### 3. Update sendMessage Function

**File**: `src/components/ChatWidget/services/chatService.ts`

Update `sendMessage` to accept and include user context:

```typescript
export async function sendMessage(
  message: string,
  sessionId: string,
  userId?: string,
  includeUserContext: boolean = false, // NEW PARAMETER
): Promise<ChatResponse> {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), REQUEST_TIMEOUT);

  try {
    // Fetch user context if user is authenticated and context is requested
    let userContext: UserContext | undefined;
    if (userId && includeUserContext) {
      userContext = await fetchUserContext(userId);
    }

    const requestBody: ChatRequest = {
      message,
      session_id: sessionId,
      ...(userId && { user_id: userId }),
      ...(userContext && { user_context: userContext }), // NEW: Include user context
    };

    // ... rest of existing code ...
  } catch (error) {
    // ... existing error handling ...
  }
}
```

### 4. Update useChatSession Hook

**File**: `src/components/ChatWidget/hooks/useChatSession.ts`

Add tracking for context sent per session:

```typescript
export function useChatSession() {
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [contextSentForSessions, setContextSentForSessions] = useState<Set<string>>(new Set());

  // ... existing code ...

  const hasContextBeenSent = useCallback((id: string): boolean => {
    return contextSentForSessions.has(id);
  }, [contextSentForSessions]);

  const markContextAsSent = useCallback((id: string): void => {
    setContextSentForSessions(prev => new Set(prev).add(id));
  }, []);

  return {
    sessionId,
    saveSession,
    clearSession,
    getSession,
    initializeSession,
    hasContextBeenSent, // NEW
    markContextAsSent, // NEW
    storageError,
  };
}
```

### 5. Update ChatWidget Component

**File**: `src/components/ChatWidget/index.tsx`

Update to fetch and send context on first message:

```typescript
import { useAuthContext } from '@site/src/components/Auth/AuthProvider';

export default function ChatWidget(): ReactNode {
  const { user } = useAuthContext(); // NEW: Get user from auth context
  const {sessionId, saveSession, hasContextBeenSent, markContextAsSent} = useChatSession();
  // ... existing code ...

  const handleSendMessage = async (messageText: string) => {
    // ... existing validation ...

    const currentSessionId = sessionId || crypto.randomUUID();
    const isFirstMessage = !hasContextBeenSent(currentSessionId);

    // Send message with user ID and context (only on first message)
    const response = await apiSendMessage(
      messageText,
      currentSessionId,
      user?.id, // Pass user ID if authenticated
      isFirstMessage, // Include context only on first message
    );

    if (isFirstMessage) {
      markContextAsSent(currentSessionId);
    }

    // ... rest of existing code ...
  };
}
```

## Backend Setup

### 1. Update ChatRequest Model

**File**: `Chatbot/src/chatbot/api/routes.py`

Add user_context field to ChatRequest:

```python
from pydantic import BaseModel, Field
from typing import Optional, List, Dict

class ProgressContext(BaseModel):
    completed_modules: Optional[List[str]] = None
    completed_sections: Optional[List[Dict[str, str]]] = None
    progress_summary: Optional[Dict[str, Dict[str, float]]] = None

class BookmarkContext(BaseModel):
    module_id: str
    section_id: str
    title: Optional[str] = None

class NoteContext(BaseModel):
    module_id: str
    section_id: str
    content: str

class UserContext(BaseModel):
    progress: Optional[ProgressContext] = None
    bookmarks: Optional[List[BookmarkContext]] = None
    notes: Optional[List[NoteContext]] = None

class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=2000)
    session_id: str
    thread_id: Optional[str] = None
    user_id: Optional[str] = None
    user_context: Optional[UserContext] = None  # NEW FIELD
```

### 2. Update Chat Endpoint

**File**: `Chatbot/src/chatbot/api/routes.py`

Incorporate user context into agent prompt:

```python
@router.post("/api/chat")
async def chat_endpoint(
    request: ChatRequest,
    agent: AgentDep,
    stream: bool = Query(False),
):
    # ... existing validation ...

    # Build context string for the agent if user_context is provided
    enhanced_message = request.message
    if request.user_context:
        context_prompt = build_context_prompt(request.user_context)
        enhanced_message = f"{context_prompt}\n\nUser question: {request.message}"

    # Use enhanced_message with the agent
    result = await Runner.run(agent, enhanced_message)

    # ... rest of existing code ...

def build_context_prompt(user_context: UserContext) -> str:
    """Build context prompt from user context data."""
    parts = []
    
    if user_context.progress:
        if user_context.progress.completed_modules:
            modules = ", ".join(user_context.progress.completed_modules)
            parts.append(f"User has completed modules: {modules}")
        if user_context.progress.completed_sections:
            parts.append(f"User has completed {len(user_context.progress.completed_sections)} sections")
    
    if user_context.bookmarks:
        parts.append(f"User has bookmarked {len(user_context.bookmarks)} sections")
    
    if user_context.notes:
        parts.append(f"User has {len(user_context.notes)} notes")
    
    return "\n".join(parts) if parts else ""
```

## Testing

### Frontend Testing

1. **Test context fetching**:
   - Authenticate as user with progress/bookmarks/notes
   - Send first message in new session
   - Verify user_context is included in request

2. **Test session tracking**:
   - Send first message (context should be included)
   - Send second message in same session (context should NOT be included)
   - Start new session (context should be included again)

3. **Test graceful degradation**:
   - Simulate API failure for progress/bookmarks/notes
   - Verify chatbot still works without context
   - Verify no errors shown to user

### Backend Testing

1. **Test with user_context**:
   - Send request with user_context field
   - Verify backend processes request successfully
   - Verify response is personalized

2. **Test without user_context** (backward compatibility):
   - Send request without user_context field
   - Verify backend processes request normally
   - Verify no errors

3. **Test empty user_context**:
   - Send request with empty user_context object
   - Verify backend handles gracefully

## Verification

1. **Check frontend**:
   - User context fetched on first message only
   - Context not sent on subsequent messages
   - Works for unauthenticated users

2. **Check backend**:
   - user_context field accepted in requests
   - Context incorporated into responses
   - Backward compatible (works without user_context)

3. **Check performance**:
   - Context fetching adds <500ms to response time
   - Context sent only once per session

## Next Steps

After implementation:
1. Test with real user data
2. Monitor performance metrics
3. Gather user feedback on personalized responses
4. Consider enhancements (e.g., context refresh on significant data changes)

