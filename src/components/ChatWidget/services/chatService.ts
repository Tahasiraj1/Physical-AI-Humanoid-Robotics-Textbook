/**
 * API service layer for chat widget
 * Handles communication with FastAPI backend
 */

import type {ChatRequest, ChatResponse, ErrorInfo, UserContext} from '../types';
import { progressService } from '@site/src/components/Personalization/services/progressService';
import { bookmarkService } from '@site/src/components/Personalization/services/bookmarkService';
import { noteService } from '@site/src/components/Personalization/services/noteService';

/**
 * Base URL for the FastAPI backend
 * Production: Hugging Face Spaces deployment
 * Development: Local FastAPI server
 */
const API_BASE_URL = 'https://tahasiraj1-humanoid-robotics-chatbot.hf.space'

/**
 * Request timeout duration in milliseconds (30 seconds)
 */
const REQUEST_TIMEOUT = 30000;

/**
 * Fetch user personalization data for chatbot context
 * Returns undefined if user is not authenticated or fetch fails
 *
 * @param userId - User ID for authenticated users
 * @returns Promise resolving to UserContext or undefined
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
      const completedModules = new Set<string>(completedSections.map((p: any) => p.moduleId as string));
      
      context.progress = {
        completedModules: Array.from(completedModules),
        completedSections: completedSections.map((p: any) => ({
          moduleId: p.moduleId,
          sectionId: p.sectionId,
        })),
        progressSummary: progress.summary || {},
      };
    } else {
      console.warn('[chatService] Failed to fetch progress data:', progressData.reason);
    }

    // Process bookmarks
    if (bookmarksData.status === 'fulfilled') {
      context.bookmarks = bookmarksData.value.bookmarks?.map((b: any) => ({
        moduleId: b.moduleId,
        sectionId: b.sectionId,
        title: b.title,
      })) || [];
    } else {
      console.warn('[chatService] Failed to fetch bookmarks data:', bookmarksData.reason);
    }

    // Process notes (limit content length to avoid large payloads)
    if (notesData.status === 'fulfilled') {
      context.notes = notesData.value.notes?.map((n: any) => ({
        moduleId: n.moduleId,
        sectionId: n.sectionId,
        content: n.content.substring(0, 200), // Limit to 200 characters (T021)
      })) || [];
    } else {
      console.warn('[chatService] Failed to fetch notes data:', notesData.reason);
    }

    return context;
  } catch (error) {
    console.warn('[chatService] Failed to fetch user context:', error);
    return undefined; // Return undefined if fetch fails - chatbot will work without context
  }
}

/**
 * Send a chat message to the backend and receive a response
 *
 * @param message - User message text (1-2000 characters)
 * @param sessionId - Session identifier (UUID)
 * @param userId - Optional user ID for authenticated users
 * @param includeUserContext - Whether to include user personalization data (default: false)
 * @returns Promise resolving to ChatResponse or rejecting with ErrorInfo
 * @throws ErrorInfo if request fails
 */
export async function sendMessage(
  message: string,
  sessionId: string,
  userId?: string,
  includeUserContext: boolean = false,
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
      ...(userContext && { user_context: userContext }),
    };

    const response = await fetch(`${API_BASE_URL}/api/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    // Handle HTTP errors
    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      const errorInfo: ErrorInfo = categorizeError(response.status, errorData);
      throw errorInfo;
    }

    // Parse and validate response
    const data: ChatResponse = await response.json();
    validateResponse(data);

    return data;
  } catch (error) {
    clearTimeout(timeoutId);

    // Handle AbortError (timeout)
    if (error instanceof Error && error.name === 'AbortError') {
      const timeoutError: ErrorInfo = {
        type: 'timeout',
        message: 'Request timed out',
        retryable: true,
        timestamp: new Date().toISOString(),
      };
      throw timeoutError;
    }

    // Handle network errors (offline, connection refused, etc.)
    if (
      error instanceof TypeError &&
      (error.message.includes('fetch') ||
        error.message.includes('Failed to fetch') ||
        error.message.includes('NetworkError'))
    ) {
      // Check if browser is offline
      if (!navigator.onLine) {
        const networkError: ErrorInfo = {
          type: 'network',
          message: 'You are offline. Please check your internet connection.',
          retryable: true,
          timestamp: new Date().toISOString(),
        };
        throw networkError;
      }

      const networkError: ErrorInfo = {
        type: 'network',
        message: 'Network connection lost',
        retryable: true,
        timestamp: new Date().toISOString(),
      };
      throw networkError;
    }

    // Re-throw ErrorInfo if already categorized
    if (isErrorInfo(error)) {
      throw error;
    }

    // Unknown error
    const unknownError: ErrorInfo = {
      type: 'network',
      message: 'An unexpected error occurred',
      retryable: true,
      timestamp: new Date().toISOString(),
    };
    throw unknownError;
  }
}

/**
 * Categorize HTTP error responses into ErrorInfo
 */
function categorizeError(status: number, errorData: any): ErrorInfo {
  if (status >= 400 && status < 500) {
    return {
      type: '4xx',
      message: errorData?.error?.message || 'Invalid request',
      code: status,
      retryable: false,
      timestamp: new Date().toISOString(),
    };
  }

  if (status >= 500) {
    return {
      type: '5xx',
      message: errorData?.error?.message || 'Server error',
      code: status,
      retryable: true,
      timestamp: new Date().toISOString(),
    };
  }

  return {
    type: 'network',
    message: 'An error occurred',
    code: status,
    retryable: true,
    timestamp: new Date().toISOString(),
  };
}

/**
 * Validate API response structure
 * Throws ErrorInfo if response is malformed
 */
function validateResponse(data: any): asserts data is ChatResponse {
  if (!data || typeof data !== 'object') {
    throw {
      type: 'validation' as const,
      message: 'Invalid response format',
      retryable: true,
      timestamp: new Date().toISOString(),
    };
  }

  if (typeof data.response !== 'string') {
    throw {
      type: 'validation' as const,
      message: 'Invalid response format: missing response field',
      retryable: true,
      timestamp: new Date().toISOString(),
    };
  }

  if (typeof data.session_id !== 'string') {
    throw {
      type: 'validation' as const,
      message: 'Invalid response format: missing session_id field',
      retryable: true,
      timestamp: new Date().toISOString(),
    };
  }

  if (!Array.isArray(data.citations)) {
    throw {
      type: 'validation' as const,
      message: 'Invalid response format: citations must be an array',
      retryable: true,
      timestamp: new Date().toISOString(),
    };
  }
}

/**
 * Type guard to check if error is ErrorInfo
 */
function isErrorInfo(error: unknown): error is ErrorInfo {
  return (
    typeof error === 'object' &&
    error !== null &&
    'type' in error &&
    'message' in error &&
    'retryable' in error
  );
}

