/**
 * ChatWidget - Main component
 * Floating chat widget for textbook Q&A
 */

import type {ReactNode} from 'react';
import React, {useState, useEffect} from 'react';
import ChatButton from './ChatButton';
import ChatWindow from './ChatWindow';
import MessageList from './MessageList';
import MessageInput from './MessageInput';
import LoadingIndicator from './LoadingIndicator';
import ErrorMessage from './ErrorMessage';
import {useChatSession} from './hooks/useChatSession';
import {useChatAPI} from './hooks/useChatAPI';
import {useMessageValidation} from './hooks/useMessageValidation';
import {useAuthContext} from '@site/src/components/Auth/AuthProvider';
import type {ChatMessage, ErrorInfo} from './types';
import styles from './styles.module.css';

export default function ChatWidget(): ReactNode {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [error, setError] = useState<ErrorInfo | null>(null);

  const {user} = useAuthContext();
  const {sessionId, saveSession, hasContextBeenSent, markContextAsSent, clearContextTracking} = useChatSession();
  const {sendMessage: apiSendMessage, isLoading: apiLoading, error: apiError, clearError: clearApiError} = useChatAPI();
  const validation = useMessageValidation();

  // Clear context tracking when user changes (FR-011)
  const previousUserIdRef = React.useRef<string | null>(null);
  useEffect(() => {
    if (user?.id) {
      // User is authenticated - context tracking will be managed per session
      // If user ID changes, clear tracking to ensure new user's context is sent
      if (previousUserIdRef.current !== null && previousUserIdRef.current !== user.id) {
        clearContextTracking();
      }
      previousUserIdRef.current = user.id;
    } else {
      // User logged out - clear tracking
      if (previousUserIdRef.current !== null) {
        clearContextTracking();
      }
      previousUserIdRef.current = null;
    }
  }, [user?.id, clearContextTracking]);

  // Widget always starts closed on new page load (per FR-011)
  // State is reset on mount, but messages persist if session exists
  useEffect(() => {
    // Widget starts closed - no need to restore open state
    // Session will be restored by useChatSession hook
    // Messages are maintained in component state and persist across widget open/close
    // Conversation history is maintained by backend via sessionId
    // Performance: Widget initialization should complete within 5 seconds (SC-001)
    // sessionStorage retrieval is synchronous and fast (< 1ms typically)
  }, []);

  const toggleWidget = () => {
    setIsOpen(!isOpen);
    // Clear error when toggling
    if (error) {
      setError(null);
    }
    // Messages state persists across open/close (per US3)
    // Widget always starts closed on new page load (per FR-011)
  };

  const handleSendMessage = async (messageText: string) => {
    if (!messageText.trim()) {
      return;
    }

    // Prevent duplicate sends while loading
    if (apiLoading) {
      return;
    }

    // Create user message
    const userMessage: ChatMessage = {
      id: crypto.randomUUID(),
      text: messageText,
      sender: 'user',
      timestamp: new Date().toISOString(),
      status: 'sending',
    };

    // Add user message immediately
    setMessages((prev) => [...prev, userMessage]);
    setError(null);
    clearApiError();

    try {
      // Use existing sessionId from sessionStorage or create new one on first message
      // Session ID persists across page navigations via sessionStorage (per US3)
      const currentSessionId = sessionId || crypto.randomUUID();
      
      // Check if this is the first message of a new session (context not sent yet)
      const isFirstMessage = !hasContextBeenSent(currentSessionId);

      // Send message to backend using hook with user context on first message
      const response = await apiSendMessage(
        messageText,
        currentSessionId,
        user?.id,
        isFirstMessage && !!user?.id, // Include context only on first message and if authenticated
      );

      // Mark context as sent for this session after successful send
      if (isFirstMessage && user?.id) {
        markContextAsSent(currentSessionId);
      }

      // Save session ID if this is the first message or if backend returned different ID
      // This ensures session persistence across page navigations (per US3)
      if (!sessionId || response.session_id !== sessionId) {
        saveSession(response.session_id);
      }

      // Update user message status
      setMessages((prev) =>
        prev.map((msg) =>
          msg.id === userMessage.id ? {...msg, status: 'sent'} : msg,
        ),
      );

      // Add assistant response
      const assistantMessage: ChatMessage = {
        id: crypto.randomUUID(),
        text: response.response,
        sender: 'assistant',
        timestamp: new Date().toISOString(),
        status: 'sent',
        citations: response.citations || [],
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err) {
      // Handle error
      const errorInfo = err as ErrorInfo;

      // Update user message with error status
      setMessages((prev) =>
        prev.map((msg) =>
          msg.id === userMessage.id
            ? {
                ...msg,
                status: 'error',
                error: errorInfo,
              }
            : msg,
        ),
      );

      setError(errorInfo);
    }
  };

  const handleRetry = () => {
    setError(null);
    // Retry last failed message if any
    const lastMessage = messages[messages.length - 1];
    if (lastMessage && lastMessage.status === 'error' && lastMessage.text) {
      handleSendMessage(lastMessage.text);
    }
  };

  return (
    <div className={styles.chatWidgetContainer}>
      {!isOpen && <ChatButton isOpen={isOpen} onClick={toggleWidget} />}
      <ChatWindow isOpen={isOpen}>
        <div className={styles.chatWindowHeader}>
          <h3>Ask about the Textbook</h3>
          <button
            className={styles.closeButton}
            onClick={toggleWidget}
            aria-label="Close chat widget"
            type="button">
            <svg
              width="20"
              height="20"
              viewBox="0 0 24 24"
              fill="none"
              xmlns="http://www.w3.org/2000/svg"
              aria-hidden="true">
              <path
                d="M19 9l-7 7-7-7"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
          </button>
        </div>
        <div className={styles.chatWindowBody}>
          <MessageList messages={messages} />
          {apiLoading && <LoadingIndicator />}
          {(error || apiError) && (
            <ErrorMessage
              error={error || apiError!}
              onRetry={handleRetry}
            />
          )}
        </div>
        <div className={styles.chatWindowFooter}>
          <MessageInput
            onSend={handleSendMessage}
            disabled={apiLoading}
            isLoading={apiLoading}
            validation={validation}
          />
        </div>
      </ChatWindow>
    </div>
  );
}

