import React, { useState, useEffect } from 'react';
import { bookmarkService } from './services/bookmarkService';
import Link from '@docusaurus/Link';
import styles from './BookmarksList.module.css';

interface Bookmark {
  id: string;
  moduleId: string;
  sectionId: string;
  createdAt: string;
}

export default function BookmarksList() {
  const [bookmarks, setBookmarks] = useState<Bookmark[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    bookmarkService.getBookmarks()
      .then((data) => setBookmarks(data.bookmarks || []))
      .catch(console.error)
      .finally(() => setLoading(false));
  }, []);

  if (loading) return <div>Loading bookmarks...</div>;
  if (bookmarks.length === 0) return <div>No bookmarks yet.</div>;

  // Group by module
  const grouped = bookmarks.reduce((acc, bookmark) => {
    if (!acc[bookmark.moduleId]) acc[bookmark.moduleId] = [];
    acc[bookmark.moduleId].push(bookmark);
    return acc;
  }, {} as Record<string, Bookmark[]>);

  return (
    <div className={styles.bookmarksList}>
      {Object.entries(grouped).map(([moduleId, moduleBookmarks]) => (
        <div key={moduleId} className={styles.moduleGroup}>
          <h3 className={styles.moduleTitle}>{moduleId}</h3>
          <ul className={styles.bookmarkList}>
            {moduleBookmarks.map((bookmark) => (
              <li key={bookmark.id}>
                <Link to={`/modules/${bookmark.moduleId}/${bookmark.sectionId}`}>
                  {bookmark.sectionId}
                </Link>
              </li>
            ))}
          </ul>
        </div>
      ))}
    </div>
  );
}

