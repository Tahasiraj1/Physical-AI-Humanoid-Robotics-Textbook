import { createAuthClient } from 'better-auth/react';

let authClientInstance: ReturnType<typeof createAuthClient> | null = null;

/**
 * Get auth URL from various sources (window, meta tag, or default)
 * This function does NOT use React hooks so it can be called from anywhere
 * Note: process.env is not available in browser, so we only use it server-side
 */
function getAuthUrl(): string {
  if (typeof window === 'undefined') {
    // Server-side only: use process.env
    return (typeof process !== 'undefined' && process.env?.AUTH_URL) || 'http://localhost:3000';
  }

  // Client-side: Try to get from window (could be set by Docusaurus or other code)
  if ((window as any).__AUTH_URL__) {
    return (window as any).__AUTH_URL__;
  }

  // Try to get from meta tag
  const metaTag = document.querySelector('meta[name="auth-url"]');
  if (metaTag) {
    const content = metaTag.getAttribute('content');
    if (content) {
      return content;
    }
  }

  // Default fallback for client-side
  return 'http://localhost:3000';
}

/**
 * Get Better Auth client instance configured with auth URL
 * This can be called from anywhere, but prefers Docusaurus context if available
 */
export function getAuthClient() {
  if (typeof window === 'undefined') {
    // Server-side: return a default client
    if (!authClientInstance) {
      const serverAuthUrl = (typeof process !== 'undefined' && process.env?.AUTH_URL) || 'http://localhost:3000';
      authClientInstance = createAuthClient({
        baseURL: serverAuthUrl,
      });
    }
    return authClientInstance;
  }

  // Client-side: get URL and create/update client
  const authUrl = getAuthUrl();
  
  if (!authClientInstance || authClientInstance.baseURL !== authUrl) {
    console.log('[getAuthClient] Creating auth client with baseURL:', authUrl);
    authClientInstance = createAuthClient({
      baseURL: authUrl,
      fetchOptions: {
        credentials: 'include', // Important for cookies to work
      },
    });
  }
  
  return authClientInstance;
}

/**
 * Auth client instance - use getAuthClient() in components instead
 * This is exported for convenience but getAuthClient() is preferred
 */
export const authClient = typeof window !== 'undefined' 
  ? createAuthClient({
      baseURL: (window as any).__AUTH_URL__ || 'http://localhost:3000',
    })
  : createAuthClient({
      baseURL: 'http://localhost:3000',
    });

