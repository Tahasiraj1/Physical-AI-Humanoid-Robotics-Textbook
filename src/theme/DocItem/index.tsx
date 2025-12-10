/**
 * Swizzled DocItem component to add personalization features
 * This wraps the original DocItem and adds BookmarkButton, NoteEditor, and CommentSection
 */

import React from 'react';
import DocItem from '@theme-original/DocItem';
import type {Props} from '@theme/DocItem';
import {useLocation} from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import BookmarkButton from '@site/src/components/Personalization/BookmarkButton';
import NoteEditor from '@site/src/components/Personalization/NoteEditor';
import CommentSection from '@site/src/components/Personalization/CommentSection';
import styles from './styles.module.css';

/**
 * Extracts module ID and section ID from document path
 * Document paths are typically in format: "/Physical-AI-Humanoid-Robotics-Textbook/modules/module-1-ros2-nervous-system/introduction"
 */
function extractIds(pathname: string, baseUrl: string): {moduleId: string; sectionId: string} {
  // Remove baseUrl and leading/trailing slashes
  // baseUrl is like "/Physical-AI-Humanoid-Robotics-Textbook/"
  const normalizedBaseUrl = baseUrl.replace(/\/$/, ''); // Remove trailing slash
  let cleanPath = pathname;
  
  // Remove baseUrl if present
  if (normalizedBaseUrl && pathname.startsWith(normalizedBaseUrl)) {
    cleanPath = pathname.slice(normalizedBaseUrl.length);
  }
  
  // Remove leading/trailing slashes
  cleanPath = cleanPath.replace(/^\/+|\/+$/g, '');
  
  // Handle different path formats
  const parts = cleanPath.split('/').filter(Boolean);
  
  // If it starts with "modules/", extract module name
  if (parts[0] === 'modules' && parts.length >= 2) {
    const moduleId = parts[1]; // e.g., "module-1-ros2-nervous-system"
    const sectionId = parts.slice(1).join('/'); // Full path after "modules/"
    return {moduleId, sectionId};
  }
  
  // If it's a simple path like "intro", use it as both
  if (parts.length === 1) {
    return {moduleId: parts[0], sectionId: parts[0]};
  }
  
  // Default: use first part as module, full path as section
  return {
    moduleId: parts[0] || cleanPath || 'default',
    sectionId: cleanPath || 'default',
  };
}

export default function DocItemWrapper(props: Props): JSX.Element {
  const location = useLocation();
  const {siteConfig} = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl || '/';
  const {moduleId, sectionId} = extractIds(location.pathname, baseUrl);

  return (
    <>
      <DocItem {...props} />
      <div className={styles.personalizationSection}>
        <div className={styles.personalizationHeader}>
          <h2>Personalize This Section</h2>
        </div>
        <div className={styles.personalizationActions}>
          <div className={styles.actionItem}>
            <BookmarkButton moduleId={moduleId} sectionId={sectionId} />
          </div>
        </div>
        <div className={styles.personalizationContent}>
          <div className={styles.contentSection}>
            <NoteEditor moduleId={moduleId} sectionId={sectionId} />
          </div>
          <div className={styles.contentSection}>
            <CommentSection moduleId={moduleId} sectionId={sectionId} />
          </div>
        </div>
      </div>
    </>
  );
}

