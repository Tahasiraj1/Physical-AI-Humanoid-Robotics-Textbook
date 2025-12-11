import React, { useState, useEffect } from 'react';
import { useAuthContext } from '@site/src/components/Auth/AuthProvider';
import { bookmarkService } from './services/bookmarkService';
import Link from '@docusaurus/Link';
import styles from './BookmarkButton.module.css';

interface BookmarkButtonProps {
  moduleId: string;
  sectionId: string;
}

export default function BookmarkButton({ moduleId, sectionId }: BookmarkButtonProps) {
  const { user } = useAuthContext();
  const [isBookmarked, setIsBookmarked] = useState(false);
  const [bookmarkId, setBookmarkId] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    if (user) {
      bookmarkService.checkBookmark(moduleId, sectionId)
        .then((data) => {
          setIsBookmarked(data.isBookmarked);
          setBookmarkId(data.bookmark?.id || null);
        })
        .catch(console.error);
    }
  }, [user, moduleId, sectionId]);

  const handleClick = async () => {
    if (!user) {
      // Prompt to sign in
      if (typeof window !== 'undefined') {
        const signIn = confirm('Please sign in to bookmark this section. Would you like to sign in now?');
        if (signIn) {
          window.location.href = `/signin?returnUrl=${encodeURIComponent(window.location.pathname)}`;
        }
      }
      return;
    }

    setLoading(true);
    try {
      if (isBookmarked && bookmarkId) {
        await bookmarkService.deleteBookmark(bookmarkId);
        setIsBookmarked(false);
        setBookmarkId(null);
      } else {
        const result = await bookmarkService.createBookmark(moduleId, sectionId);
        setIsBookmarked(true);
        setBookmarkId(result.bookmark?.id || null);
      }
    } catch (error: any) {
      console.error('Bookmark error:', error);
      
      // Provide more detailed error message
      let errorMessage = 'Failed to update bookmark. Please try again.';
      
      // Check for specific error status codes
      if (error.status === 500) {
        errorMessage = 'Server error. The database tables may not exist. Please run the database migrations (see progress.md for instructions).';
      } else if (error.status === 401) {
        errorMessage = 'Please sign in to bookmark sections.';
      } else if (error.status === 409) {
        // Bookmark already exists, just update state
        setIsBookmarked(true);
        // Try to get the existing bookmark
        try {
          const checkResult = await bookmarkService.checkBookmark(moduleId, sectionId);
          if (checkResult.isBookmarked && checkResult.bookmark) {
            setBookmarkId(checkResult.bookmark.id);
          }
        } catch (e) {
          console.error('Error checking existing bookmark:', e);
        }
        return; // Don't show error for this case
      } else if (error.message) {
        if (error.message.includes('Failed to create bookmark')) {
          errorMessage = 'Failed to create bookmark. Make sure you are signed in and the server is running.';
        } else if (error.message.includes('Failed to delete bookmark')) {
          errorMessage = 'Failed to delete bookmark. Please try again.';
        } else if (error.message.includes('NetworkError') || error.message.includes('fetch') || error.message.includes('Failed to fetch')) {
          errorMessage = 'Network error. Please check your connection and try again.';
        } else {
          errorMessage = `Error: ${error.message}`;
        }
      }
      
      alert(errorMessage);
    } finally {
      setLoading(false);
    }
  };

  if (!user) {
    return (
      <Link to="/signin" className={styles.bookmarkButton}>
        ⭐ Bookmark
      </Link>
    );
  }

  return (
    <button
      onClick={handleClick}
      disabled={loading}
      className={`${styles.bookmarkButton} ${isBookmarked ? styles.bookmarked : ''}`}
      title={isBookmarked ? 'Remove bookmark' : 'Bookmark this section'}
    >
      {isBookmarked ? '⭐ Bookmarked' : '☆ Bookmark'}
    </button>
  );
}

