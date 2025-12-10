/**
 * Custom navbar item component that conditionally shows sign-in/sign-up or user profile
 * This can be used by swizzling the NavbarItem component
 */
import React from 'react';
import { AuthProvider, useAuthContext } from '@site/src/components/Auth/AuthProvider';
import Link from '@docusaurus/Link';

function AuthNavbarContent() {
  const { user, loading, signOut } = useAuthContext();

  if (loading) {
    return null;
  }

  if (!user) {
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

  return (
    <>
      <Link to="/dashboard" className="navbar__item navbar__link" title={user.email}>
        {user.name || user.email}
      </Link>
      <button
        onClick={() => signOut()}
        className="navbar__item navbar__link"
        style={{ 
          background: 'none', 
          border: 'none', 
          cursor: 'pointer',
          color: 'inherit',
          font: 'inherit'
        }}
      >
        Sign Out
      </button>
    </>
  );
}

export default function AuthNavbarItem() {
  return (
    <AuthProvider>
      <AuthNavbarContent />
    </AuthProvider>
  );
}

