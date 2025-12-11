import React, { useState, useEffect } from 'react';
import { getAuthUrl } from '@site/src/lib/auth-url';
import styles from './DownloadHistory.module.css';

interface Download {
  id: string;
  resourceId: string;
  downloadedAt: string;
  resource?: {
    fileName: string;
    moduleId: string | null;
  };
}

export default function DownloadHistory() {
  const [downloads, setDownloads] = useState<Download[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Fetch from dashboard data
    const authUrl = getAuthUrl();
    
    fetch(`${authUrl}/api/personalization/dashboard`, {
      credentials: 'include',
    })
      .then((res) => res.json())
      .then((data) => {
        // Download history would be in dashboard data
        setDownloads([]); // Placeholder - would need separate endpoint
      })
      .catch(console.error)
      .finally(() => setLoading(false));
  }, []);

  if (loading) return <div>Loading download history...</div>;
  if (downloads.length === 0) return <div>No downloads yet.</div>;

  return (
    <div className={styles.downloadHistory}>
      <ul className={styles.downloadsList}>
        {downloads.map((download) => (
          <li key={download.id} className={styles.downloadItem}>
            <span className={styles.fileName}>
              {download.resource?.fileName || 'Unknown file'}
            </span>
            <span className={styles.downloadDate}>
              {new Date(download.downloadedAt).toLocaleDateString()}
            </span>
          </li>
        ))}
      </ul>
    </div>
  );
}

