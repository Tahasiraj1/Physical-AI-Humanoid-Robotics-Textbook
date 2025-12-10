import React, { useState, FormEvent } from 'react';
import { getAuthClient } from '@site/src/lib/auth-client';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './Auth.module.css';

interface SignInProps {
  onSuccess?: () => void;
}

/**
 * SignIn component with email and password input fields
 */
export default function SignIn({ onSuccess }: SignInProps) {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const { siteConfig } = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl || '';

  const authClient = getAuthClient();

  /**
   * Handle form submission
   */
  const handleSubmit = async (e: FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    setError(null);
    setLoading(true);

    try {
      const result = await authClient.signIn.email({
        email,
        password,
      });

      console.log('Sign in result:', result); // Debug log

      // Check for error in response (Better Auth returns { data, error })
      if (result?.error) {
        // Invalid credentials - use generic error message (security best practice)
        setError('Invalid email or password. Please try again.');
        setLoading(false);
        // Stay on page - no redirect on error
        return;
      }

      // Success - user is signed in
      // Trigger navbar update immediately, then redirect after a short delay
      if (typeof window !== 'undefined') {
        console.log('[SignIn] Sign-in successful, triggering navbar update...');
        
        // Dispatch event immediately to trigger navbar refresh
        window.dispatchEvent(new CustomEvent('auth-state-change'));
        
        // Also trigger a storage event (Better Auth uses localStorage)
        window.dispatchEvent(new StorageEvent('storage', {
          key: 'better-auth.session',
          newValue: 'updated',
        }));
        
        // Verify session and redirect
        const verifyAndRedirect = async () => {
          try {
            // Wait a bit for cookie to be set
            await new Promise(resolve => setTimeout(resolve, 300));
            
            const session = await authClient.getSession();
            if (session?.data?.user || session?.user) {
              console.log('[SignIn] Session verified, redirecting...');
              // Dispatch one more event before redirect
              window.dispatchEvent(new CustomEvent('auth-state-change'));
              
              if (onSuccess) {
                onSuccess();
              } else {
                // Use full page reload to ensure navbar updates
                window.location.href = `${baseUrl}dashboard`;
              }
            } else {
              // Session not ready yet, but redirect anyway (navbar will update on next page)
              console.log('[SignIn] Session not immediately available, redirecting anyway...');
              if (onSuccess) {
                onSuccess();
              } else {
                window.location.href = `${baseUrl}dashboard`;
              }
            }
          } catch (err) {
            console.error('[SignIn] Error verifying session:', err);
            // Redirect anyway
            if (onSuccess) {
              onSuccess();
            } else {
              window.location.href = `${baseUrl}dashboard`;
            }
          }
        };
        
        // Start verification after short delay
        setTimeout(verifyAndRedirect, 200);
      }
    } catch (err: any) {
      console.error('Sign in error:', err);
      // Use generic error message (security best practice - don't reveal if email exists)
      setError('Invalid email or password. Please try again.');
      setLoading(false);
      // Stay on page - no redirect on error
    }
  };

  return (
    <div className={styles.authContainer}>
      <h2>Sign In</h2>
      <form onSubmit={handleSubmit} className={styles.authForm}>
        <div className={styles.formGroup}>
          <label htmlFor="email">Email</label>
          <input
            id="email"
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            placeholder="your.email@example.com"
            required
            className={styles.input}
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="password">Password</label>
          <input
            id="password"
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            placeholder="Enter your password"
            required
            className={styles.input}
          />
        </div>

        {error && (
          <div className={styles.error}>
            {error}
            <div className={styles.errorActions}>
              <Link to={`${baseUrl}signup`} className={styles.errorLink}>
                Sign up
              </Link>
              <span className={styles.errorSeparator}>•</span>
              <Link to={`${baseUrl}forgot-password`} className={styles.errorLink}>
                Forgot Password
              </Link>
            </div>
          </div>
        )}

        <button type="submit" disabled={loading} className={styles.button}>
          {loading ? (
            <>
              <span className={styles.buttonSpinner}></span>
              Signing In...
            </>
          ) : (
            'Sign In'
          )}
        </button>
      </form>

      <p className={styles.footer}>
        Don't have an account? <Link to="/signup">Sign up</Link>
      </p>
    </div>
  );
}

