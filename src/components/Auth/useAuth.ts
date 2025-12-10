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
  session: any | null;
  loading: boolean;
  error: string | null;
}

/**
 * Custom hook to access Better Auth session and user data
 */
export function useAuth(): AuthState {
  const [state, setState] = useState<AuthState>({
    user: null,
    session: null,
    loading: true,
    error: null,
  });

  useEffect(() => {
    const authClient = getAuthClient();
    
    // Get initial session
    authClient.getSession()
      .then((session) => {
        setState({
          user: session?.user || null,
          session: session || null,
          loading: false,
          error: null,
        });
      })
      .catch((error) => {
        console.error('Error getting session:', error);
        setState({
          user: null,
          session: null,
          loading: false,
          error: error.message || 'Failed to get session',
        });
      });

    // Listen for auth state changes
    // Better Auth client may provide event listeners
    // For now, we'll poll or use a subscription if available
  }, []);

  return state;
}

