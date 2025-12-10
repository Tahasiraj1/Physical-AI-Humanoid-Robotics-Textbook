import React, { useEffect, useRef, useState } from 'react';
import { useHistory } from '@docusaurus/router';
import { useAuthContext } from '@site/src/components/Auth/AuthProvider';
import styles from './AvatarDropdown.module.css';

interface AvatarDropdownProps {
  isOpen: boolean;
  onClose: () => void;
}

/**
 * Dropdown menu component for avatar with Profile, Settings, and Logout options
 * Handles outside-click detection and auto-closes after menu item selection
 */
export function AvatarDropdown({ isOpen, onClose }: AvatarDropdownProps) {
  const dropdownRef = useRef<HTMLDivElement>(null);
  const history = useHistory();
  const { signOut } = useAuthContext();

  // Handle outside click to close dropdown
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        onClose();
      }
    }

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => {
        document.removeEventListener('mousedown', handleClickOutside);
      };
    }
  }, [isOpen, onClose]);

  // Handle keyboard navigation (Enter to open, Escape to close, Arrow keys to navigate)
  const [focusedIndex, setFocusedIndex] = useState<number>(-1);
  const menuItemsRef = useRef<(HTMLButtonElement | null)[]>([]);

  useEffect(() => {
    function handleKeyDown(event: KeyboardEvent) {
      if (!isOpen) return;

      if (event.key === 'Escape') {
        onClose();
        setFocusedIndex(-1);
      } else if (event.key === 'ArrowDown') {
        event.preventDefault();
        setFocusedIndex((prev) => {
          const next = prev < 2 ? prev + 1 : 0;
          menuItemsRef.current[next]?.focus();
          return next;
        });
      } else if (event.key === 'ArrowUp') {
        event.preventDefault();
        setFocusedIndex((prev) => {
          const next = prev > 0 ? prev - 1 : 2;
          menuItemsRef.current[next]?.focus();
          return next;
        });
      } else if (event.key === 'Home') {
        event.preventDefault();
        setFocusedIndex(0);
        menuItemsRef.current[0]?.focus();
      } else if (event.key === 'End') {
        event.preventDefault();
        setFocusedIndex(2);
        menuItemsRef.current[2]?.focus();
      }
    }

    if (isOpen) {
      document.addEventListener('keydown', handleKeyDown);
      // Focus first item when dropdown opens
      setTimeout(() => {
        menuItemsRef.current[0]?.focus();
        setFocusedIndex(0);
      }, 0);
      return () => {
        document.removeEventListener('keydown', handleKeyDown);
      };
    } else {
      setFocusedIndex(-1);
    }
  }, [isOpen, onClose]);

  const handleProfileClick = () => {
    history.push('/profile');
    onClose();
  };

  const handleSettingsClick = () => {
    history.push('/profile#settings');
    // Scroll to settings section if hash is present
    setTimeout(() => {
      const settingsElement = document.getElementById('settings');
      if (settingsElement) {
        settingsElement.scrollIntoView({ behavior: 'smooth' });
      }
    }, 100);
    onClose();
  };

  const handleLogoutClick = async () => {
    try {
      await signOut();
      history.push('/');
      onClose();
    } catch (error) {
      console.error('Error signing out:', error);
    }
  };

  if (!isOpen) return null;

  return (
    <div 
      ref={dropdownRef} 
      className={styles.dropdown}
      role="menu"
      aria-label="User menu"
    >
      <button
        ref={(el) => (menuItemsRef.current[0] = el)}
        className={styles.menuItem}
        onClick={handleProfileClick}
        role="menuitem"
        tabIndex={focusedIndex === 0 ? 0 : -1}
      >
        Profile
      </button>
      <button
        ref={(el) => (menuItemsRef.current[1] = el)}
        className={styles.menuItem}
        onClick={handleSettingsClick}
        role="menuitem"
        tabIndex={focusedIndex === 1 ? 0 : -1}
      >
        Settings
      </button>
      <button
        ref={(el) => (menuItemsRef.current[2] = el)}
        className={styles.menuItem}
        onClick={handleLogoutClick}
        role="menuitem"
        tabIndex={focusedIndex === 2 ? 0 : -1}
      >
        Logout
      </button>
    </div>
  );
}

