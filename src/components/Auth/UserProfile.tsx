import React, { useState } from 'react';
import { useAuthContext } from './AuthProvider';
import Link from '@docusaurus/Link';
import styles from './UserProfile.module.css';

/**
 * UserProfile component to display user name and email with update form
 */
export default function UserProfile() {
  const { user, signOut, loading, refreshSession } = useAuthContext();
  const [isEditing, setIsEditing] = useState(false);
  const [name, setName] = useState(user?.name || '');
  const [learningLevel, setLearningLevel] = useState('beginner');
  const [saving, setSaving] = useState(false);

  const handleSave = async () => {
    setSaving(true);
    try {
      const authUrl = typeof window !== 'undefined' 
        ? (window as any).__AUTH_URL__ || 'http://localhost:3000'
        : 'http://localhost:3000';
      
      const response = await fetch(`${authUrl}/api/personalization/profile`, {
        method: 'PATCH',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({ name, learningLevel }),
      });

      if (response.ok) {
        setIsEditing(false);
        await refreshSession();
      } else {
        throw new Error('Failed to update profile');
      }
    } catch (error) {
      console.error('Profile update error:', error);
      alert('Failed to update profile. Please try again.');
    } finally {
      setSaving(false);
    }
  };

  if (loading) {
    return <div className={styles.profile}>Loading...</div>;
  }

  if (!user) {
    return (
      <div className={styles.profile}>
        <Link to="/signin" className={styles.link}>Sign In</Link>
        <Link to="/signup" className={styles.link}>Sign Up</Link>
      </div>
    );
  }

  return (
    <div className={styles.profile}>
      {isEditing ? (
        <div className={styles.editForm}>
          <input
            type="text"
            value={name}
            onChange={(e) => setName(e.target.value)}
            placeholder="Your name"
            className={styles.input}
          />
          <select
            value={learningLevel}
            onChange={(e) => setLearningLevel(e.target.value)}
            className={styles.select}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
          <div className={styles.formActions}>
            <button onClick={handleSave} disabled={saving} className={styles.saveButton}>
              {saving ? 'Saving...' : 'Save'}
            </button>
            <button onClick={() => setIsEditing(false)} className={styles.cancelButton}>
              Cancel
            </button>
          </div>
        </div>
      ) : (
        <>
          <div className={styles.userInfo}>
            <span className={styles.name}>{user.name || user.email}</span>
            {user.name && <span className={styles.email}>{user.email}</span>}
          </div>
          <div className={styles.actions}>
            <Link to="/dashboard" className={styles.link}>Dashboard</Link>
            <button onClick={() => setIsEditing(true)} className={styles.editButton}>
              Edit Profile
            </button>
            <button onClick={() => signOut()} className={styles.signOutButton}>
              Sign Out
            </button>
          </div>
        </>
      )}
    </div>
  );
}
