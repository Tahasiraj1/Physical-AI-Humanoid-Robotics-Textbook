import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { getAuthClient, getAuthUrlString } from '@site/src/lib/auth-client';
import type { User } from './useAuth';

interface AuthContextType {
  user: User | null;
  session: any | null;
  loading: boolean;
  signOut: () => Promise<void>;
  refreshSession: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

/**
 * AuthProvider component to wrap app with Better Auth context
 * Provides authentication state and methods to child components
 */
export function AuthProvider({ children }: AuthProviderProps) {
  const [user, setUser] = useState<User | null>(null);
  const [session, setSession] = useState<any | null>(null);
  const [loading, setLoading] = useState(true);
  // Track previous user ID to detect actual changes
  const previousUserIdRef = React.useRef<string | null>(null);

  const authClient = getAuthClient();

  /**
   * Check session on page load and set up session persistence
   */
  useEffect(() => {
    let isMounted = true;
    let retryCount = 0;
    const maxRetries = 15; // Try for up to 7.5 seconds (15 * 500ms)

    const checkSession = async (isRetry = false) => {
      if (!isMounted) return;
      
      try {
        if (!isRetry) {
          console.log('[AuthProvider] Checking session...'); // Debug log
          // Better Auth client's baseURL might be a getter, try to access it safely
          try {
            const baseURL = typeof authClient.baseURL === 'string' ? authClient.baseURL : 
                          (authClient as any).baseURL || 'unknown';
            console.log('[AuthProvider] Auth client baseURL:', baseURL);
          } catch (e) {
            console.log('[AuthProvider] Auth client baseURL: (could not access)');
          }
        }
        
        let currentSession: any;
        
        // Try Better Auth client's getSession first
        try {
          currentSession = await authClient.getSession();
        } catch (clientError) {
          console.warn('[AuthProvider] getSession() failed, trying direct API call:', clientError);
          // Fallback: Try direct API call
          try {
            const authUrl = getAuthUrlString();
            const response = await fetch(`${authUrl}/api/auth/get-session`, {
              credentials: 'include',
              headers: {
                'Content-Type': 'application/json',
              },
            });
            
            if (response.ok) {
              currentSession = await response.json();
              console.log('[AuthProvider] Direct API call succeeded:', currentSession);
            } else {
              console.warn('[AuthProvider] Direct API call failed:', response.status, response.statusText);
              currentSession = null;
            }
          } catch (apiError) {
            console.error('[AuthProvider] Both getSession() and direct API call failed:', apiError);
            currentSession = null;
          }
        }
        
        // Log the full response for debugging
        if (!isRetry) {
          console.log('[AuthProvider] Session response:', JSON.stringify(currentSession, null, 2));
        }
        
        // Better Auth returns { data: { user, session }, error: null }
        // But it might also return the user/session directly
        const user = currentSession?.data?.user || currentSession?.user || null;
        const session = currentSession?.data?.session || currentSession?.session || null;
        
        // Also check for error in response
        const error = currentSession?.error || currentSession?.data?.error;
        if (error) {
          console.warn('[AuthProvider] Session check returned error:', error);
        }
        
        if (user) {
          console.log('[AuthProvider] ✅ User found:', user.email, 'User ID:', user.id); // Debug log
          if (isMounted) {
            const newUserId = user.id;
            const userChanged = previousUserIdRef.current !== newUserId;
            
            // CRITICAL: Only call setUser if user actually changed to prevent unnecessary re-renders
            if (userChanged) {
            setUser(user);
            setSession(session);
            previousUserIdRef.current = newUserId;
              // Only emit event if user actually changed (not on every check)
              window.dispatchEvent(new CustomEvent('auth-state-change'));
            } else {
              // User hasn't changed, just update the ref
              previousUserIdRef.current = newUserId;
            }
            
            // Always set loading to false once we have a user (even if unchanged)
            setLoading(false);
          }
        } else {
          if (!isRetry) {
            console.log('[AuthProvider] ❌ No user in session. Full response:', currentSession); // Debug log
            // Check if cookies are present
            const cookies = document.cookie;
            console.log('[AuthProvider] Current cookies:', cookies);
            const hasAuthCookie = cookies.includes('better-auth') || cookies.includes('session');
            console.log('[AuthProvider] Has auth-related cookies:', hasAuthCookie);
          }
          
          // Retry checking session if we haven't exceeded max retries
          // This is important after sign-in when cookie might not be immediately available
          if (retryCount < maxRetries && isMounted) {
            retryCount++;
            setTimeout(() => {
              checkSession(true);
            }, 500);
          } else if (isMounted) {
            setUser(null);
            setSession(null);
            setLoading(false);
          }
        }
      } catch (error) {
        console.error('[AuthProvider] ❌ Error checking session:', error);
        console.error('[AuthProvider] Error details:', {
          message: error instanceof Error ? error.message : String(error),
          stack: error instanceof Error ? error.stack : undefined,
        });
        if (isMounted) {
          // Still retry on error
          if (retryCount < maxRetries) {
            retryCount++;
            setTimeout(() => {
              checkSession(true);
            }, 500);
          } else {
            setUser(null);
            setSession(null);
            setLoading(false);
          }
        }
      }
    };

    checkSession();

    // Set up periodic session refresh (every 5 minutes)
    const interval = setInterval(checkSession, 5 * 60 * 1000);
    
    // Set up localStorage interceptor only once (singleton pattern)
    if (!(window as any).__AUTH_LOCALSTORAGE_INTERCEPTED__) {
      (window as any).__AUTH_LOCALSTORAGE_INTERCEPTED__ = true;
      const originalSetItem = localStorage.setItem.bind(localStorage);
      localStorage.setItem = function(key: string, value: string) {
        originalSetItem(key, value);
        // Only emit local-storage-change, NOT auth-state-change (to prevent infinite loops)
        // The checkSession handler will check and emit auth-state-change only if state changed
        if (key?.includes('auth') || key?.includes('session') || key?.includes('better-auth')) {
          window.dispatchEvent(new CustomEvent('local-storage-change', { 
            detail: { key, value } 
          }));
          // Don't emit auth-state-change here - let checkSession handle it
        }
      };

      const originalRemoveItem = localStorage.removeItem.bind(localStorage);
      localStorage.removeItem = function(key: string) {
        originalRemoveItem(key);
        // Only emit local-storage-change, NOT auth-state-change (to prevent infinite loops)
        // The checkSession handler will check and emit auth-state-change only if state changed
        if (key?.includes('auth') || key?.includes('session') || key?.includes('better-auth')) {
          window.dispatchEvent(new CustomEvent('local-storage-change', { 
            detail: { key, value: null } 
          }));
          // Don't emit auth-state-change here - let checkSession handle it
        }
      };
    }
    
    // Also listen for storage events (for cross-tab sync)
    const handleStorageChange = () => {
      checkSession();
    };
    window.addEventListener('storage', handleStorageChange);

    // Listen for custom auth state change events
    // CRITICAL: Check flag to prevent infinite loops when refreshSession emits events
    const handleAuthStateChange = () => {
      // Don't call checkSession if refreshSession is currently running
      // This prevents: refreshSession() → emit event → checkSession() → emit event → refreshSession() → loop
      if (!(window as any).__AUTH_REFRESHING__) {
      checkSession();
      } else {
        console.log('[AuthProvider] Ignoring auth-state-change event (refreshSession in progress)');
      }
    };
    window.addEventListener('auth-state-change', handleAuthStateChange);

    // Listen for same-tab localStorage changes
    const handleLocalStorageChange = () => {
      checkSession();
    };
    window.addEventListener('local-storage-change', handleLocalStorageChange);
    
    return () => {
      isMounted = false;
      clearInterval(interval);
      // Note: We don't restore localStorage methods here because they're shared
      // and other AuthProvider instances might still be using them
      window.removeEventListener('storage', handleStorageChange);
      window.removeEventListener('auth-state-change', handleAuthStateChange);
      window.removeEventListener('local-storage-change', handleLocalStorageChange);
    };
  }, []);

  /**
   * Sign out functionality
   */
  const signOut = async () => {
    try {
      await authClient.signOut();
      setUser(null);
      setSession(null);
      // Redirect to home page after sign out
      if (typeof window !== 'undefined') {
        // Get baseUrl from current location
        const baseUrl = window.location.pathname.split('/').slice(0, -1).join('/') || '';
        window.location.href = baseUrl || '/';
      }
    } catch (error) {
      console.error('Error signing out:', error);
      throw error;
    }
  };

  /**
   * Refresh session - memoized to prevent infinite loops
   * Only emits events when user state actually changes
   * CRITICAL: Do NOT include user in dependencies - use ref instead to prevent infinite loops
   */
  const refreshSession = React.useCallback(async () => {
    // Set flag to prevent checkSession from running when we emit events
    const wasRefreshing = (window as any).__AUTH_REFRESHING__ || false;
    (window as any).__AUTH_REFRESHING__ = true;
    
    try {
      console.log('[AuthProvider] Refreshing session...');
      let currentSession: any;
      
      // Try Better Auth client's getSession first
      try {
        currentSession = await authClient.getSession();
      } catch (clientError) {
        console.warn('[AuthProvider] Refresh: getSession() failed, trying direct API call:', clientError);
        // Fallback: Try direct API call
        try {
          const authUrl = getAuthUrlString();
          const response = await fetch(`${authUrl}/api/auth/get-session`, {
            credentials: 'include',
            headers: {
              'Content-Type': 'application/json',
            },
          });
          
          if (response.ok) {
            currentSession = await response.json();
            console.log('[AuthProvider] Refresh: Direct API call succeeded');
          } else {
            console.warn('[AuthProvider] Refresh: Direct API call failed:', response.status);
            currentSession = null;
          }
        } catch (apiError) {
          console.error('[AuthProvider] Refresh: Both methods failed:', apiError);
          currentSession = null;
        }
      }
      
      // Better Auth returns { data: { user, session }, error: null }
      const newUser = currentSession?.data?.user || currentSession?.user || null;
      const newSession = currentSession?.data?.session || currentSession?.session || null;
      
      // Check if user state actually changed before updating and emitting events
      const newUserId = newUser?.id || null;
      const userChanged = previousUserIdRef.current !== newUserId;
      
      if (newUser) {
        console.log('[AuthProvider] ✅ Refresh: User found:', newUser.email);
        // CRITICAL: Only call setUser if user actually changed to prevent unnecessary re-renders
        if (userChanged) {
        setUser(newUser);
        setSession(newSession);
        previousUserIdRef.current = newUserId;
          console.log('[AuthProvider] User state changed, emitting auth-state-change event');
          window.dispatchEvent(new CustomEvent('auth-state-change'));
        } else {
          // User hasn't changed, but update refs to keep them in sync
          previousUserIdRef.current = newUserId;
          // Don't call setUser() - prevents unnecessary re-renders and context value recreation
        }
      } else {
        console.log('[AuthProvider] ❌ Refresh: No user found');
        // Only update and emit if we had a user before
        if (previousUserIdRef.current !== null) {
          setUser(null);
          setSession(null);
          previousUserIdRef.current = null;
          window.dispatchEvent(new CustomEvent('auth-state-change'));
        }
        // If previousUserIdRef was already null, do nothing (no change)
      }
    } catch (error) {
      console.error('[AuthProvider] ❌ Error refreshing session:', error);
      // Use ref to check previous state instead of user prop
      if (previousUserIdRef.current !== null) {
        setUser(null);
        setSession(null);
        previousUserIdRef.current = null;
        window.dispatchEvent(new CustomEvent('auth-state-change'));
      } else {
        setUser(null);
        setSession(null);
      }
    } finally {
      // Clear the flag after a short delay to allow event handlers to see it
      setTimeout(() => {
        (window as any).__AUTH_REFRESHING__ = false;
      }, 100);
    }
  }, [authClient]); // CRITICAL: Do NOT include user - causes infinite loops

  const value: AuthContextType = {
    user,
    session,
    loading,
    signOut,
    refreshSession,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

/**
 * Hook to use auth context
 */
export function useAuthContext(): AuthContextType {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuthContext must be used within an AuthProvider');
  }
  return context;
}

