import { useState, useEffect } from 'react';
import { getAuthClient } from '@site/src/lib/auth-client';

export interface User {
  id: string;
  name: string | null;
  email: string;
  emailVerified: boolean;
  image: string | null;
}

export interface AuthState {
  user: User | null;
  isAuthenticated: boolean;
  loading: boolean;
  signOut: () => Promise<void>;
}

/**
 * Custom hook that wraps Better Auth client SDK for session management
 * Provides simplified interface for authentication state
 */
export function useAuth(): AuthState {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const authClient = getAuthClient();
    
    // Get initial session
    authClient.getSession()
      .then((session) => {
        setUser(session?.user || null);
        setLoading(false);
      })
      .catch((error) => {
        console.error('Error getting session:', error);
        setUser(null);
        setLoading(false);
      });

    // TODO: Set up session change listeners if Better Auth provides them
  }, []);

  const signOut = async (): Promise<void> => {
    const authClient = getAuthClient();
    try {
      await authClient.signOut();
      setUser(null);
      // Redirect handled by Better Auth or caller
    } catch (error) {
      console.error('Error signing out:', error);
      throw error;
    }
  };

  return {
    user,
    isAuthenticated: !!user,
    loading,
    signOut,
  };
}

