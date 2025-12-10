import React from 'react';
import { AuthProvider, useAuthContext } from '@site/src/components/Auth/AuthProvider';
import Link from '@docusaurus/Link';
import styles from './UserProfileNavbarItem.module.css';

function UserProfileNavbarItemContent() {
  const { user, loading } = useAuthContext();

  if (loading) {
    return null; // Don't show anything while loading
  }

  if (!user) {
    // Show sign-in and sign-up links
    return (
      <>
        <Link to="/signin" className="navbar__item navbar__link">
          Sign In
        </Link>
        <Link to="/signup" className="navbar__item navbar__link">
          Sign Up
        </Link>
      </>
    );
  }

  // Show user profile with dropdown
  return (
    <div className={styles.userProfile}>
      <Link to="/dashboard" className="navbar__item navbar__link">
        <span className={styles.userName}>{user.name || user.email}</span>
      </Link>
    </div>
  );
}

export default function UserProfileNavbarItem() {
  return (
    <AuthProvider>
      <UserProfileNavbarItemContent />
    </AuthProvider>
  );
}

