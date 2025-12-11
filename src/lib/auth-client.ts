import { createAuthClient } from "better-auth/react";

let authClientInstance: ReturnType<typeof createAuthClient> | null = null;
let currentAuthUrl: string | null = null; // Store the URL separately to avoid Proxy issues

// Hardcoded production auth URL
const PRODUCTION_AUTH_URL = "https://physical-ai-auth-service-l2tf.vercel.app";
const LOCAL_AUTH_URL = "http://localhost:3000";

/**
 * Get auth URL - hardcoded for production, localhost for development
 * This function does NOT use React hooks so it can be called from anywhere
 */
function getAuthUrl(): string {
  if (typeof window === "undefined") {
    // Server-side only: use process.env
    return (
      (typeof process !== "undefined" && process.env?.AUTH_URL) ||
      LOCAL_AUTH_URL
    );
  }

  // Detect if we're in production (GitHub Pages) or local development
  const isLocalhost =
    window.location.hostname === "localhost" ||
    window.location.hostname === "127.0.0.1" ||
    window.location.hostname.includes("localhost");

  // Production: always use hardcoded Vercel URL
  if (!isLocalhost) {
    return PRODUCTION_AUTH_URL;
  }

  // Local development: Try to get from window (set by Docusaurus Root component)
  if ((window as any).__AUTH_URL__) {
    return (window as any).__AUTH_URL__;
  }

  // Try to get from meta tag
  const metaTag = document.querySelector('meta[name="auth-url"]');
  if (metaTag) {
    const content = metaTag.getAttribute("content");
    if (content) {
      return content;
    }
  }

  // Default fallback for local development
  return LOCAL_AUTH_URL;
}

/**
 * Get Better Auth client instance configured with auth URL
 * This can be called from anywhere, but prefers Docusaurus context if available
 */
export function getAuthClient() {
  if (typeof window === "undefined") {
    // Server-side: return a default client
    if (!authClientInstance) {
      const serverAuthUrl =
        (typeof process !== "undefined" && process.env?.AUTH_URL) ||
        LOCAL_AUTH_URL;
      currentAuthUrl = serverAuthUrl;
      authClientInstance = createAuthClient({
        baseURL: serverAuthUrl,
      });
    }
    return authClientInstance;
  }

  // Client-side: get URL and create/update client
  const authUrl = getAuthUrl();

  // Compare stored URL instead of accessing baseURL Proxy
  if (!authClientInstance || currentAuthUrl !== authUrl) {
    console.log("[getAuthClient] Creating auth client with baseURL:", authUrl);
    currentAuthUrl = authUrl;
    authClientInstance = createAuthClient({
      baseURL: authUrl,
      fetchOptions: {
        credentials: "include", // Important for cookies to work
      },
    });
  }

  return authClientInstance;
}

/**
 * Get the current auth URL (use this instead of accessing baseURL directly)
 * This avoids Proxy conversion issues
 */
export function getAuthUrlString(): string {
  return currentAuthUrl || getAuthUrl();
}

/**
 * Auth client instance - use getAuthClient() in components instead
 * This is exported for convenience but getAuthClient() is preferred
 */
export const authClient =
  typeof window !== "undefined"
    ? createAuthClient({
        baseURL: (window as any).__AUTH_URL__ || PRODUCTION_AUTH_URL,
      })
    : createAuthClient({
        baseURL: LOCAL_AUTH_URL,
      });
