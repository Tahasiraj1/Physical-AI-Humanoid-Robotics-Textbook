import React from 'react';
import styles from './ProgressIndicator.module.css';

interface ProgressIndicatorProps {
  moduleId: string;
  progressPercentage: number;
  viewedSections?: number;
  totalSections?: number;
}

/**
 * ProgressIndicator component to display progress percentage for a module
 */
export default function ProgressIndicator({
  moduleId,
  progressPercentage,
  viewedSections,
  totalSections,
}: ProgressIndicatorProps) {
  return (
    <div className={styles.progressContainer}>
      <div className={styles.progressHeader}>
        <span className={styles.moduleName}>{moduleId}</span>
        <span className={styles.percentage}>{progressPercentage}%</span>
      </div>
      <div className={styles.progressBar}>
        <div
          className={styles.progressFill}
          style={{ width: `${progressPercentage}%` }}
        />
      </div>
      {viewedSections !== undefined && totalSections !== undefined && (
        <div className={styles.progressDetails}>
          {viewedSections} of {totalSections} sections viewed
        </div>
      )}
    </div>
  );
}

