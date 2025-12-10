import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import styles from './UserCommentsList.module.css';

interface Comment {
  id: string;
  moduleId: string;
  sectionId: string;
  content: string;
  createdAt: string;
}

export default function UserCommentsList() {
  const [comments, setComments] = useState<Comment[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Fetch user's comments from dashboard data
    const authUrl = typeof window !== 'undefined' 
      ? (window as any).__AUTH_URL__ || 'http://localhost:3000'
      : 'http://localhost:3000';
    
    fetch(`${authUrl}/api/personalization/dashboard`, {
      credentials: 'include',
    })
      .then((res) => res.json())
      .then((data) => setComments(data.comments || []))
      .catch(console.error)
      .finally(() => setLoading(false));
  }, []);

  if (loading) return <div>Loading comments...</div>;
  if (comments.length === 0) return <div>No comments yet.</div>;

  return (
    <div className={styles.commentsList}>
      {comments.map((comment) => (
        <div key={comment.id} className={styles.commentItem}>
          <div className={styles.commentHeader}>
            <Link to={`/modules/${comment.moduleId}/${comment.sectionId}`} className={styles.commentLink}>
              {comment.moduleId} / {comment.sectionId}
            </Link>
            <span className={styles.commentDate}>
              {new Date(comment.createdAt).toLocaleDateString()}
            </span>
          </div>
          <div className={styles.commentContent}>
            {comment.content.substring(0, 150)}
            {comment.content.length > 150 ? '...' : ''}
          </div>
        </div>
      ))}
    </div>
  );
}

