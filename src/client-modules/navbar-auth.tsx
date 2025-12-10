/**
 * Client module to inject user profile into navbar
 * This runs on the client side and modifies the navbar based on auth state
 */
import React from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { AuthProvider, useAuthContext } from '@site/src/components/Auth/AuthProvider';
import Link from '@docusaurus/Link';

function UserProfileNavbarItem() {
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
    <div style={{ display: 'flex', alignItems: 'center', gap: '1rem' }}>
      <Link to="/dashboard" className="navbar__item navbar__link">
        {user.name || user.email}
      </Link>
      <button
        onClick={() => signOut()}
        className="navbar__item navbar__link"
        style={{ background: 'none', border: 'none', cursor: 'pointer', padding: '0.5rem' }}
      >
        Sign Out
      </button>
    </div>
  );
}

export default function NavbarAuth() {
  if (!ExecutionEnvironment.canUseDOM) {
    return null;
  }

  // Wait for DOM to be ready
  React.useEffect(() => {
    const injectUserProfile = () => {
      const navbar = document.querySelector('.navbar__items--right');
      const signInItem = Array.from(navbar?.children || []).find(
        (el: any) => el.textContent?.includes('Sign In')
      );
      const signUpItem = Array.from(navbar?.children || []).find(
        (el: any) => el.textContent?.includes('Sign Up')
      );

      if (signInItem && signUpItem) {
        // Create container for user profile
        const container = document.createElement('div');
        container.id = 'user-profile-navbar-container';
        signInItem.replaceWith(container);
        signUpItem.remove();

        // Render user profile component
        const root = document.createElement('div');
        container.appendChild(root);
        
        // This is a simplified version - in production, use ReactDOM.render
        // For now, we'll handle this differently
      }
    };

    // Try to inject after a delay to ensure navbar is rendered
    setTimeout(injectUserProfile, 100);
  }, []);

  return null;
}

