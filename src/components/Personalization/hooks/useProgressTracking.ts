import { useEffect } from 'react';
import { useAuthContext } from '@site/src/components/Auth/AuthProvider';
import { progressService } from '../services/progressService';

/**
 * Hook to track section views on page load
 * Extracts moduleId and sectionId from the current page URL
 */
export function useProgressTracking() {
  const { user } = useAuthContext();

  useEffect(() => {
    if (!user) return; // Only track for authenticated users

    // Extract module and section from URL
    // Docusaurus URLs: /modules/module-1-ros2-nervous-system/introduction
    const path = typeof window !== 'undefined' ? window.location.pathname : '';
    const match = path.match(/\/modules\/([^/]+)\/([^/]+)/);
    
    if (match) {
      const [, moduleId, sectionId] = match;
      progressService.recordSectionView(moduleId, sectionId).catch((error) => {
        console.error('Failed to track progress:', error);
      });
    }
  }, [user]);
}

