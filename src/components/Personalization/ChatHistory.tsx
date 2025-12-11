import React, { useState, useEffect } from 'react';
import { getAuthUrl } from '@site/src/lib/auth-url';
import styles from './ChatHistory.module.css';

interface ChatSession {
  id: string;
  sessionId: string;
  messages: any[];
  startedAt: string;
  lastActivityAt: string;
}

export default function ChatHistory() {
  const [sessions, setSessions] = useState<ChatSession[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const authUrl = getAuthUrl();
    
    fetch(`${authUrl}/api/personalization/chat-sessions`, {
      credentials: 'include',
    })
      .then((res) => res.json())
      .then((data) => setSessions(data.sessions || []))
      .catch(console.error)
      .finally(() => setLoading(false));
  }, []);

  if (loading) return <div>Loading chat history...</div>;
  if (sessions.length === 0) return <div>No chat history yet.</div>;

  return (
    <div className={styles.chatHistory}>
      <h3>Chat History</h3>
      <ul className={styles.sessionsList}>
        {sessions.map((session) => (
          <li key={session.id} className={styles.sessionItem}>
            <div className={styles.sessionHeader}>
              <span className={styles.sessionId}>Session: {session.sessionId.substring(0, 8)}...</span>
              <span className={styles.sessionDate}>
                {new Date(session.lastActivityAt).toLocaleDateString()}
              </span>
            </div>
            <div className={styles.messageCount}>
              {session.messages?.length || 0} messages
            </div>
          </li>
        ))}
      </ul>
    </div>
  );
}

