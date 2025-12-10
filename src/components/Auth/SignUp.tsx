import React, { useState, FormEvent, useEffect } from 'react';
import { getAuthClient } from '@site/src/lib/auth-client';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './Auth.module.css';

interface SignUpProps {
  onSuccess?: () => void;
}

interface Avatar {
  id: string;
  imageUrl: string;
  displayName: string | null;
}

// Predefined option lists for personalization questions
const SOFTWARE_OPTIONS = [
  'ROS 2',
  'Gazebo',
  'Python',
  'ROS',
  'Gazebo Sim',
  'RViz',
  'MoveIt',
  'OpenCV',
];

const HARDWARE_OPTIONS = [
  'Humanoid robots',
  'Manipulators',
  'Sensors',
  'Actuators',
  'Mobile robots',
  'Drones',
];

const PROGRAMMING_LANGUAGE_OPTIONS = [
  'Python',
  'C++',
  'JavaScript',
  'TypeScript',
  'Rust',
  'Go',
];

/**
 * SignUp component with email, password, name, avatar selection, and personalization questions
 */
export default function SignUp({ onSuccess }: SignUpProps) {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [selectedAvatarId, setSelectedAvatarId] = useState<string | null>(null);
  const [softwarePreferences, setSoftwarePreferences] = useState<string[]>([]);
  const [hardwarePreferences, setHardwarePreferences] = useState<string[]>([]);
  const [programmingLanguagePreferences, setProgrammingLanguagePreferences] = useState<string[]>([]);
  const [avatars, setAvatars] = useState<Avatar[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const [loadingAvatars, setLoadingAvatars] = useState(true);
  const { siteConfig } = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl || '';
  const authUrl = (siteConfig.customFields?.authUrl as string) || 'http://localhost:3000';

  const authClient = getAuthClient();

  // Fetch available avatars on component mount
  useEffect(() => {
    const fetchAvatars = async () => {
      try {
        const response = await fetch(`${authUrl}/api/avatar/list`, {
          credentials: 'include',
        });
        if (response.ok) {
          const data = await response.json();
          setAvatars(data.avatars || []);
        }
      } catch (err) {
        console.error('Error fetching avatars:', err);
      } finally {
        setLoadingAvatars(false);
      }
    };
    fetchAvatars();
  }, [authUrl]);

  // Toggle preference selection
  const togglePreference = (
    preference: string,
    currentPreferences: string[],
    setPreferences: (prefs: string[]) => void
  ) => {
    if (currentPreferences.includes(preference)) {
      setPreferences(currentPreferences.filter((p) => p !== preference));
    } else {
      setPreferences([...currentPreferences, preference]);
    }
  };

  /**
   * Email validation
   */
  const validateEmail = (email: string): boolean => {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  };

  /**
   * Password validation
   */
  const validatePassword = (password: string): { valid: boolean; message?: string } => {
    if (password.length < 8) {
      return { valid: false, message: 'Password must be at least 8 characters long' };
    }
    if (!/(?=.*[a-z])/.test(password)) {
      return { valid: false, message: 'Password must contain at least one lowercase letter' };
    }
    if (!/(?=.*[A-Z])/.test(password)) {
      return { valid: false, message: 'Password must contain at least one uppercase letter' };
    }
    if (!/(?=.*\d)/.test(password)) {
      return { valid: false, message: 'Password must contain at least one number' };
    }
    return { valid: true };
  };

  /**
   * Handle form submission
   */
  const handleSubmit = async (e: FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    setError(null);

    // Validate email
    if (!validateEmail(email)) {
      setError('Please enter a valid email address');
      return;
    }

    // Validate password
    const passwordValidation = validatePassword(password);
    if (!passwordValidation.valid) {
      setError(passwordValidation.message || 'Invalid password');
      return;
    }

    setLoading(true);

    try {
      const result = await authClient.signUp.email({
        email,
        password,
        name: name || undefined,
      });

      console.log('Sign up result:', result); // Debug log

      // Check for error in response (Better Auth returns { data, error })
      if (result?.error) {
        // Account already exists or other error
        const errorMessage = result.error.message || result.error.toString();
        if (errorMessage.toLowerCase().includes('email') || 
            errorMessage.toLowerCase().includes('already') || 
            errorMessage.toLowerCase().includes('exists') ||
            errorMessage.toLowerCase().includes('duplicate') ||
            errorMessage.toLowerCase().includes('unique') ||
            result.error.code === 'ACCOUNT_EXISTS') {
          setError('An account with this email already exists. Please sign in or use a different email.');
        } else {
          setError(errorMessage);
        }
        setLoading(false);
        // Stay on page - no redirect on error
        return;
      }

      // Success - Better Auth automatically signs user in after sign-up
      // Verify session was created
      if (result?.data?.user && result?.data?.session) {
        console.log('User automatically signed in:', result.data.user.email);
        
        // Save avatar selection and preferences after account creation
        try {
          // Save avatar if selected
          if (selectedAvatarId) {
            await fetch(`${authUrl}/api/personalization/avatar`, {
              method: 'PUT',
              headers: { 'Content-Type': 'application/json' },
              credentials: 'include',
              body: JSON.stringify({ avatarId: selectedAvatarId }),
            });
          }

          // Save preferences if any are selected
          if (
            softwarePreferences.length > 0 ||
            hardwarePreferences.length > 0 ||
            programmingLanguagePreferences.length > 0
          ) {
            await fetch(`${authUrl}/api/personalization/preferences`, {
              method: 'PUT',
              headers: { 'Content-Type': 'application/json' },
              credentials: 'include',
              body: JSON.stringify({
                softwarePreferences: softwarePreferences.length > 0 ? softwarePreferences : null,
                hardwarePreferences: hardwarePreferences.length > 0 ? hardwarePreferences : null,
                programmingLanguagePreferences:
                  programmingLanguagePreferences.length > 0 ? programmingLanguagePreferences : null,
              }),
            });
          }
        } catch (prefError) {
          console.error('Error saving avatar/preferences:', prefError);
          // Don't block sign-up if preferences fail - account is already created
        }
      }

      // Redirect to dashboard only on success
      // Trigger navbar update immediately, then redirect after a short delay
      if (typeof window !== 'undefined') {
        console.log('[SignUp] Sign-up successful, triggering navbar update...');
        
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
              console.log('[SignUp] Session verified, redirecting...');
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
              console.log('[SignUp] Session not immediately available, redirecting anyway...');
              if (onSuccess) {
                onSuccess();
              } else {
                window.location.href = `${baseUrl}dashboard`;
              }
            }
          } catch (err) {
            console.error('[SignUp] Error verifying session:', err);
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
      console.error('Sign up error:', err);
      // Handle network errors or other exceptions
      const errorMessage = err.message || err.error?.message || err.data?.message || 'Failed to create account. Please try again.';
      if (errorMessage.toLowerCase().includes('email') || 
          errorMessage.toLowerCase().includes('already') || 
          errorMessage.toLowerCase().includes('exists') ||
          errorMessage.toLowerCase().includes('duplicate') ||
          errorMessage.toLowerCase().includes('unique')) {
        setError('An account with this email already exists. Please sign in or use a different email.');
      } else {
        setError(errorMessage);
      }
      setLoading(false);
      // Stay on page - no redirect on error
    }
  };

  return (
    <div className={styles.authContainer}>
      <h2>Create Account</h2>
      <form onSubmit={handleSubmit} className={styles.authForm}>
        <div className={styles.formGroup}>
          <label htmlFor="name">Name (optional)</label>
          <input
            id="name"
            type="text"
            value={name}
            onChange={(e) => setName(e.target.value)}
            placeholder="Your name"
            className={styles.input}
          />
        </div>

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
            placeholder="At least 8 characters with uppercase, lowercase, and number"
            required
            className={styles.input}
            minLength={8}
          />
          <small className={styles.helpText}>
            Password must be at least 8 characters with uppercase, lowercase, and number
          </small>
        </div>

        {/* Avatar Selection */}
        <div className={styles.formGroup}>
          <label>Choose an Avatar (optional)</label>
          {loadingAvatars ? (
            <div className={styles.loading}>Loading avatars...</div>
          ) : (
            <div className={styles.avatarGrid}>
              {avatars.map((avatar) => (
                <button
                  key={avatar.id}
                  type="button"
                  className={`${styles.avatarOption} ${
                    selectedAvatarId === avatar.id ? styles.avatarSelected : ''
                  }`}
                  onClick={() =>
                    setSelectedAvatarId(selectedAvatarId === avatar.id ? null : avatar.id)
                  }
                  aria-label={`Select ${avatar.displayName || avatar.id}`}
                >
                  <img src={avatar.imageUrl} alt={avatar.displayName || avatar.id} />
                </button>
              ))}
            </div>
          )}
        </div>

        {/* Personalization Questions */}
        <div className={styles.personalizationSection}>
          <h3 className={styles.personalizationTitle}>Help us personalize your experience (optional)</h3>

          {/* Software Preferences */}
          <div className={styles.formGroup}>
            <label>Which software do you use?</label>
            <div className={styles.checkboxGroup}>
              {SOFTWARE_OPTIONS.map((option) => (
                <label key={option} className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    checked={softwarePreferences.includes(option)}
                    onChange={() =>
                      togglePreference(option, softwarePreferences, setSoftwarePreferences)
                    }
                  />
                  <span>{option}</span>
                </label>
              ))}
            </div>
          </div>

          {/* Hardware Preferences */}
          <div className={styles.formGroup}>
            <label>Which hardware do you use or are interested in?</label>
            <div className={styles.checkboxGroup}>
              {HARDWARE_OPTIONS.map((option) => (
                <label key={option} className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    checked={hardwarePreferences.includes(option)}
                    onChange={() =>
                      togglePreference(option, hardwarePreferences, setHardwarePreferences)
                    }
                  />
                  <span>{option}</span>
                </label>
              ))}
            </div>
          </div>

          {/* Programming Language Preferences */}
          <div className={styles.formGroup}>
            <label>Which programming languages do you use?</label>
            <div className={styles.checkboxGroup}>
              {PROGRAMMING_LANGUAGE_OPTIONS.map((option) => (
                <label key={option} className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    checked={programmingLanguagePreferences.includes(option)}
                    onChange={() =>
                      togglePreference(
                        option,
                        programmingLanguagePreferences,
                        setProgrammingLanguagePreferences
                      )
                    }
                  />
                  <span>{option}</span>
                </label>
              ))}
            </div>
          </div>
        </div>

        {error && (
          <div className={styles.error}>
            {error}
            {error.toLowerCase().includes('already exists') && (
              <div className={styles.errorActions}>
                <Link to={`${baseUrl}signin`} className={styles.errorLink}>
                  Sign in instead
                </Link>
              </div>
            )}
          </div>
        )}

        <button type="submit" disabled={loading || loadingAvatars} className={styles.button}>
          {loading ? (
            <>
              <span className={styles.buttonSpinner}></span>
              Creating Account...
            </>
          ) : (
            'Sign Up'
          )}
        </button>
      </form>

      <p className={styles.footer}>
        Already have an account? <Link to="/signin">Sign in</Link>
      </p>
    </div>
  );
}

