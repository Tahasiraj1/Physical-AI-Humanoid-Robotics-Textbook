import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { AuthProvider, useAuthContext } from '@site/src/components/Auth/AuthProvider';
import ProgressIndicator from '@site/src/components/Personalization/ProgressIndicator';
import BookmarksList from '@site/src/components/Personalization/BookmarksList';
import NotesList from '@site/src/components/Personalization/NotesList';
import Recommendations from '@site/src/components/Personalization/Recommendations';
import UserCommentsList from '@site/src/components/Personalization/UserCommentsList';
import Link from '@docusaurus/Link';
import styles from './dashboard.module.css';

interface DashboardData {
  progress: {
    recent: any[];
    summary: Record<string, { totalSections: number; viewedSections: number; progressPercentage: number }>;
  };
  bookmarks: any[];
  notes: any[];
  comments: any[];
  downloadHistory: any[];
  recommendations: any[];
}

function DashboardContent(): JSX.Element {
  const { user, loading: authLoading } = useAuthContext();
  const [data, setData] = useState<DashboardData | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    console.log('Dashboard useEffect - authLoading:', authLoading, 'user:', user); // Debug log
    if (!authLoading && user) {
      const authUrl = typeof window !== 'undefined' 
        ? (window as any).__AUTH_URL__ || 'http://localhost:3000'
        : 'http://localhost:3000';
      
      fetch(`${authUrl}/api/personalization/dashboard`, {
        credentials: 'include',
      })
        .then((res) => {
          if (!res.ok) {
            throw new Error(`HTTP error! status: ${res.status}`);
          }
          return res.json();
        })
        .then((data) => setData(data))
        .catch((error) => {
          console.error('Dashboard fetch error:', error);
        })
        .finally(() => setLoading(false));
    } else if (!authLoading && !user) {
      console.log('No user found, setting loading to false'); // Debug log
      setLoading(false);
    }
  }, [user, authLoading]);

  if (authLoading || loading) {
    return (
      <Layout title="Dashboard" description="Your personal dashboard">
        <div className="container margin-vert--lg">
          <div>Loading...</div>
        </div>
      </Layout>
    );
  }

  if (!user) {
    return (
      <Layout title="Dashboard" description="Your personal dashboard">
        <div className="container margin-vert--lg">
          <h1>Dashboard</h1>
          <p>Please <Link to="/signin">sign in</Link> to view your dashboard.</p>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Dashboard" description="Your personal dashboard">
      <div className="container margin-vert--lg">
        <h1>Welcome, {user.name || user.email}!</h1>

        <div className={styles.dashboardGrid}>
          <section className={styles.section}>
            <h2>Progress</h2>
            {data?.progress?.summary ? (
              Object.entries(data.progress.summary).map(([moduleId, summary]) => (
                <ProgressIndicator
                  key={moduleId}
                  moduleId={moduleId}
                  progressPercentage={summary.progressPercentage}
                  viewedSections={summary.viewedSections}
                  totalSections={summary.totalSections}
                />
              ))
            ) : (
              <p>No progress tracked yet.</p>
            )}
          </section>

          <section className={styles.section}>
            <h2>Bookmarks</h2>
            <BookmarksList />
          </section>

          <section className={styles.section}>
            <h2>Notes</h2>
            <NotesList />
          </section>

          {data?.recommendations && data.recommendations.length > 0 && (
            <section className={styles.section}>
              <h2>Recommendations</h2>
              <Recommendations />
            </section>
          )}

          {data?.comments && data.comments.length > 0 && (
            <section className={styles.section}>
              <h2>Your Comments</h2>
              <UserCommentsList />
            </section>
          )}
        </div>
      </div>
    </Layout>
  );
}

export default function DashboardPage(): JSX.Element {
  return (
    <AuthProvider>
      <DashboardContent />
    </AuthProvider>
  );
}

