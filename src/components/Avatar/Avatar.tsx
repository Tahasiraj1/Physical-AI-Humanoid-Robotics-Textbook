import React, { useState, useEffect } from 'react';
import { useAuthContext } from '@site/src/components/Auth/AuthProvider';
import { FirstLetterAvatar } from './FirstLetterAvatar';
import { AvatarDropdown } from './AvatarDropdown';
import styles from './Avatar.module.css';

interface AvatarData {
  hasSelectedAvatar: boolean;
  selectedAvatarId: string | null;
  selectedAvatarImageUrl: string | null;
  userName: string;
  avatarDisplayType: 'predefined' | 'letter' | 'default';
  firstLetter: string | null;
}

/**
 * Avatar component that displays user's selected avatar or first-letter avatar
 * Handles click to toggle dropdown menu
 */
export function Avatar() {
  const { user, loading: authLoading } = useAuthContext();
  const isAuthenticated = !!user;
  const [avatarData, setAvatarData] = useState<AvatarData | null>(null);
  const [dropdownOpen, setDropdownOpen] = useState(false);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  console.log('[Avatar] 🎨 Render - user:', user?.email || 'null', 'authLoading:', authLoading, 'isAuthenticated:', isAuthenticated);

  // Fetch avatar data when user is authenticated
  useEffect(() => {
    console.log('[Avatar] 🔄 useEffect - isAuthenticated:', isAuthenticated, 'user:', user?.email || 'null');
    if (!isAuthenticated || !user) {
      console.log('[Avatar] ❌ Not authenticated or no user, skipping avatar fetch');
      setLoading(false);
      return;
    }

    const fetchAvatarData = async () => {
      try {
        setLoading(true);
        setError(null);
        
        // Get auth URL from window (set by Root component) or fallback
        const authUrl = (typeof window !== 'undefined' && (window as any).__AUTH_URL__) 
          || process.env.AUTH_URL 
          || 'http://localhost:3000';
        const response = await fetch(`${authUrl}/api/personalization/avatar`, {
          credentials: 'include',
        });

        if (!response.ok) {
          throw new Error('Failed to fetch avatar data');
        }

        const data = await response.json();
        setAvatarData(data.avatar);
      } catch (err: any) {
        console.error('Error fetching avatar data:', err);
        setError(err.message);
        // Fallback to first-letter avatar if API fails
        if (user.name) {
          const match = user.name.match(/[a-zA-Z0-9]/);
          const firstLetter = match ? match[0].toUpperCase() : '?';
          setAvatarData({
            hasSelectedAvatar: false,
            selectedAvatarId: null,
            selectedAvatarImageUrl: null,
            userName: user.name,
            avatarDisplayType: 'letter',
            firstLetter,
          });
        }
      } finally {
        setLoading(false);
      }
    };

    fetchAvatarData();
  }, [isAuthenticated, user]);

  if (!isAuthenticated || !user) {
    return null;
  }

  if (loading || authLoading) {
    return (
      <div className={styles.avatarContainer}>
        <div className={styles.loading} aria-label="Loading avatar" role="status">
          <span className={styles.loadingSpinner}></span>
        </div>
      </div>
    );
  }

  if (error && !avatarData) {
    // Show error state but still allow interaction
    console.error('Avatar loading error:', error);
  }

  const handleAvatarClick = () => {
    setDropdownOpen(!dropdownOpen);
  };

  const displayLetter = avatarData?.firstLetter || 
    (user.name ? (user.name.match(/[a-zA-Z0-9]/)?.[0].toUpperCase() || '?') : '?');

  return (
    <div className={styles.avatarContainer}>
      <button
        className={styles.avatarButton}
        onClick={handleAvatarClick}
        aria-label="User menu"
        aria-expanded={dropdownOpen}
        aria-haspopup="true"
      >
        {avatarData?.hasSelectedAvatar && avatarData.selectedAvatarImageUrl ? (
          <img
            src={avatarData.selectedAvatarImageUrl}
            alt="User avatar"
            className={styles.avatarImage}
          />
        ) : (
          <FirstLetterAvatar letter={displayLetter} size={40} />
        )}
      </button>
      <AvatarDropdown isOpen={dropdownOpen} onClose={() => setDropdownOpen(false)} />
    </div>
  );
}

