import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { AuthProvider, useAuthContext } from '@site/src/components/Auth/AuthProvider';
import { FirstLetterAvatar } from '@site/src/components/Avatar/FirstLetterAvatar';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './profile.module.css';

interface ProfileData {
  userId: string;
  name: string;
  email: string;
  learningLevel: string | null;
  selectedAvatarId: string | null;
  selectedAvatarImageUrl: string | null;
  softwarePreferences: string[];
  hardwarePreferences: string[];
  programmingLanguagePreferences: string[];
  createdAt: string;
  updatedAt: string;
}

interface Avatar {
  id: string;
  imageUrl: string;
  displayName: string | null;
}

// Same option lists as sign-up
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

function ProfileContent() {
  const { user, loading: authLoading } = useAuthContext();
  const { siteConfig } = useDocusaurusContext();
  const authUrl = (siteConfig.customFields?.authUrl as string) || 'http://localhost:3000';

  const [profileData, setProfileData] = useState<ProfileData | null>(null);
  const [avatars, setAvatars] = useState<Avatar[]>([]);
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);

  // Settings state
  const [selectedAvatarId, setSelectedAvatarId] = useState<string | null>(null);
  const [softwarePreferences, setSoftwarePreferences] = useState<string[]>([]);
  const [hardwarePreferences, setHardwarePreferences] = useState<string[]>([]);
  const [programmingLanguagePreferences, setProgrammingLanguagePreferences] = useState<string[]>([]);

  // Fetch profile data and avatars
  useEffect(() => {
    if (!user) return;

    const fetchData = async () => {
      try {
        setLoading(true);
        const [profileResponse, avatarsResponse] = await Promise.all([
          fetch(`${authUrl}/api/personalization/profile`, { credentials: 'include' }),
          fetch(`${authUrl}/api/avatar/list`, { credentials: 'include' }),
        ]);

        if (profileResponse.ok) {
          const profileData = await profileResponse.json();
          setProfileData(profileData.profile);
          // Initialize settings state from profile
          setSelectedAvatarId(profileData.profile.selectedAvatarId);
          setSoftwarePreferences(profileData.profile.softwarePreferences || []);
          setHardwarePreferences(profileData.profile.hardwarePreferences || []);
          setProgrammingLanguagePreferences(profileData.profile.programmingLanguagePreferences || []);
        }

        if (avatarsResponse.ok) {
          const avatarsData = await avatarsResponse.json();
          setAvatars(avatarsData.avatars || []);
        }
      } catch (err: any) {
        console.error('Error fetching profile data:', err);
        setError('Failed to load profile data');
      } finally {
        setLoading(false);
      }
    };

    fetchData();
  }, [user, authUrl]);

  // Scroll to settings section if hash is present
  useEffect(() => {
    if (typeof window !== 'undefined' && window.location.hash === '#settings') {
      setTimeout(() => {
        const settingsElement = document.getElementById('settings');
        if (settingsElement) {
          settingsElement.scrollIntoView({ behavior: 'smooth' });
        }
      }, 100);
    }
  }, []);

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

  const handleSaveSettings = async () => {
    try {
      setSaving(true);
      setError(null);
      setSuccess(null);

      // Save avatar
      if (selectedAvatarId !== profileData?.selectedAvatarId) {
        await fetch(`${authUrl}/api/personalization/avatar`, {
          method: 'PUT',
          headers: { 'Content-Type': 'application/json' },
          credentials: 'include',
          body: JSON.stringify({ avatarId: selectedAvatarId }),
        });
      }

      // Save preferences
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

      setSuccess('Settings saved successfully!');
      // Refresh profile data
      const profileResponse = await fetch(`${authUrl}/api/personalization/profile`, {
        credentials: 'include',
      });
      if (profileResponse.ok) {
        const profileData = await profileResponse.json();
        setProfileData(profileData.profile);
      }
    } catch (err: any) {
      console.error('Error saving settings:', err);
      setError('Failed to save settings. Please try again.');
    } finally {
      setSaving(false);
    }
  };

  if (authLoading || loading) {
    return (
      <div className="container margin-vert--lg">
        <div className={styles.loading}>Loading profile...</div>
      </div>
    );
  }

  if (!user) {
    return (
      <div className="container margin-vert--lg">
        <div className={styles.error}>Please sign in to view your profile.</div>
      </div>
    );
  }

  const displayLetter =
    profileData?.name && profileData.name.trim().length > 0
      ? (profileData.name.match(/[a-zA-Z0-9]/)?.[0].toUpperCase() || '?')
      : '?';

  return (
    <div className="container margin-vert--lg">
      <div className={styles.profileContainer}>
        {/* Profile Section */}
        <section className={styles.profileSection}>
          <h1>Profile</h1>
          <div className={styles.profileInfo}>
            <div className={styles.avatarDisplay}>
              {profileData?.selectedAvatarImageUrl ? (
                <img
                  src={profileData.selectedAvatarImageUrl}
                  alt="User avatar"
                  className={styles.avatarImage}
                />
              ) : (
                <FirstLetterAvatar letter={displayLetter} size={80} />
              )}
            </div>
            <div className={styles.userInfo}>
              <h2>{profileData?.name || profileData?.email || 'User'}</h2>
              <p className={styles.email}>{profileData?.email}</p>
              {profileData?.learningLevel && (
                <p className={styles.learningLevel}>
                  Learning Level: <strong>{profileData.learningLevel}</strong>
                </p>
              )}
            </div>
          </div>

          {/* Display Preferences (Read-only) */}
          {(profileData?.softwarePreferences?.length > 0 ||
            profileData?.hardwarePreferences?.length > 0 ||
            profileData?.programmingLanguagePreferences?.length > 0) && (
            <div className={styles.preferencesDisplay}>
              <h3>Your Preferences</h3>
              {profileData.softwarePreferences?.length > 0 && (
                <div className={styles.preferenceGroup}>
                  <strong>Software:</strong>{' '}
                  {profileData.softwarePreferences.join(', ')}
                </div>
              )}
              {profileData.hardwarePreferences?.length > 0 && (
                <div className={styles.preferenceGroup}>
                  <strong>Hardware:</strong>{' '}
                  {profileData.hardwarePreferences.join(', ')}
                </div>
              )}
              {profileData.programmingLanguagePreferences?.length > 0 && (
                <div className={styles.preferenceGroup}>
                  <strong>Programming Languages:</strong>{' '}
                  {profileData.programmingLanguagePreferences.join(', ')}
                </div>
              )}
            </div>
          )}
        </section>

        {/* Settings Section */}
        <section id="settings" className={styles.settingsSection}>
          <h1>Settings</h1>
          <p className={styles.settingsDescription}>
            Update your avatar and personalization preferences
          </p>

          {error && <div className={styles.error}>{error}</div>}
          {success && <div className={styles.success}>{success}</div>}

          {/* Avatar Selection */}
          <div className={styles.formGroup}>
            <label>Choose an Avatar</label>
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
          </div>

          {/* Personalization Questions */}
          <div className={styles.personalizationSection}>
            <h3>Personalization Preferences</h3>

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

          <button
            type="button"
            onClick={handleSaveSettings}
            disabled={saving}
            className={styles.saveButton}
          >
            {saving ? 'Saving...' : 'Save Settings'}
          </button>
        </section>
      </div>
    </div>
  );
}

export default function ProfilePage(): JSX.Element {
  return (
    <AuthProvider>
      <Layout title="Profile" description="View and edit your profile and preferences">
        <ProfileContent />
      </Layout>
    </AuthProvider>
  );
}

