import React, { useState, useEffect } from 'react';
import { useAuthContext } from '@site/src/components/Auth/AuthProvider';
import Link from '@docusaurus/Link';
import styles from './CommentSection.module.css';

interface Comment {
  id: string;
  content: string;
  name: string;
  email: string;
  createdAt: string;
  parentCommentId: string | null;
}

interface CommentSectionProps {
  moduleId: string;
  sectionId: string;
}

export default function CommentSection({ moduleId, sectionId }: CommentSectionProps) {
  const { user } = useAuthContext();
  const [comments, setComments] = useState<Comment[]>([]);
  const [newComment, setNewComment] = useState('');
  const [loading, setLoading] = useState(true);
  const [submitting, setSubmitting] = useState(false);

  const fetchComments = async () => {
    try {
      const authUrl = typeof window !== 'undefined' 
        ? (window as any).__AUTH_URL__ || 'http://localhost:3000'
        : 'http://localhost:3000';
      const response = await fetch(
        `${authUrl}/api/personalization/comments?moduleId=${moduleId}&sectionId=${sectionId}`
      );
      if (response.ok) {
        const data = await response.json();
        setComments(data.comments || []);
      }
    } catch (error) {
      console.error('Error fetching comments:', error);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    fetchComments();
  }, [moduleId, sectionId]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!user) {
      const signIn = confirm('Please sign in to post comments. Would you like to sign in now?');
      if (signIn) {
        window.location.href = `/signin?returnUrl=${encodeURIComponent(window.location.pathname)}`;
      }
      return;
    }

    if (!newComment.trim()) return;

    setSubmitting(true);
    try {
      const authUrl = typeof window !== 'undefined' 
        ? (window as any).__AUTH_URL__ || 'http://localhost:3000'
        : 'http://localhost:3000';
      const response = await fetch(`${authUrl}/api/personalization/comments`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({ moduleId, sectionId, content: newComment }),
      });

      if (response.ok) {
        setNewComment('');
        fetchComments();
      } else {
        throw new Error('Failed to post comment');
      }
    } catch (error) {
      console.error('Error posting comment:', error);
      alert('Failed to post comment. Please try again.');
    } finally {
      setSubmitting(false);
    }
  };

  if (loading) return <div>Loading comments...</div>;

  return (
    <div className={styles.commentSection}>
      <h3>Comments & Discussion</h3>
      
      {comments.length === 0 ? (
        <p>No comments yet. Be the first to comment!</p>
      ) : (
        <div className={styles.commentsList}>
          {comments.map((comment) => (
            <div key={comment.id} className={styles.comment}>
              <div className={styles.commentHeader}>
                <strong>{comment.name || comment.email}</strong>
                <span className={styles.commentDate}>
                  {new Date(comment.createdAt).toLocaleDateString()}
                </span>
              </div>
              <div className={styles.commentContent}>{comment.content}</div>
            </div>
          ))}
        </div>
      )}

      {user ? (
        <form onSubmit={handleSubmit} className={styles.commentForm}>
          <textarea
            value={newComment}
            onChange={(e) => setNewComment(e.target.value)}
            placeholder="Add a comment..."
            className={styles.textarea}
            rows={3}
            required
          />
          <button type="submit" disabled={submitting} className={styles.submitButton}>
            {submitting ? 'Posting...' : 'Post Comment'}
          </button>
        </form>
      ) : (
        <p className={styles.signInPrompt}>
          Please <Link to="/signin">sign in</Link> to post comments.
        </p>
      )}
    </div>
  );
}

