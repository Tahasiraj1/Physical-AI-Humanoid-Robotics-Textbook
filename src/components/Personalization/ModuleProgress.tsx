import React from 'react';
import ProgressIndicator from './ProgressIndicator';
import styles from './ModuleProgress.module.css';

interface ModuleProgressProps {
  moduleId: string;
  progressData: {
    totalSections: number;
    viewedSections: number;
    progressPercentage: number;
  };
}

/**
 * Module progress visualization component to show progress bars per module
 */
export default function ModuleProgress({ moduleId, progressData }: ModuleProgressProps) {
  return (
    <div className={styles.moduleProgress}>
      <ProgressIndicator
        moduleId={moduleId}
        progressPercentage={progressData.progressPercentage}
        viewedSections={progressData.viewedSections}
        totalSections={progressData.totalSections}
      />
    </div>
  );
}

