/**
 * Shared utility to get auth URL for all services
 * Hardcodes production URL, uses localhost only for development
 */

const PRODUCTION_AUTH_URL = 'https://physical-ai-auth-service-l2tf.vercel.app';
const LOCAL_AUTH_URL = 'http://localhost:3000';

/**
 * Get auth URL - hardcoded for production, localhost for development
 * This can be called from anywhere (no React hooks required)
 */
export function getAuthUrl(): string {
  if (typeof window === 'undefined') {
    return (typeof process !== 'undefined' && process.env?.AUTH_URL) || LOCAL_AUTH_URL;
  }

  // Detect if we're in production (GitHub Pages) or local development
  const isLocalhost = window.location.hostname === 'localhost' || 
                     window.location.hostname === '127.0.0.1';
  
  // Production: always use hardcoded Vercel URL
  if (!isLocalhost) {
    return PRODUCTION_AUTH_URL;
  }

  // Local development: Try to get from window or meta tag
  if ((window as any).__AUTH_URL__) {
    return (window as any).__AUTH_URL__;
  }

  const metaTag = document.querySelector('meta[name="auth-url"]');
  if (metaTag) {
    const content = metaTag.getAttribute('content');
    if (content) return content;
  }

  return LOCAL_AUTH_URL;
}

