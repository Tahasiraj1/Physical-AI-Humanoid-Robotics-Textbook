import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuthContext } from '@site/src/components/Auth/AuthProvider';
import Link from '@docusaurus/Link';
import styles from './resources.module.css';

interface Resource {
  id: string;
  fileName: string;
  fileType: string;
  fileSize: number;
  moduleId: string | null;
  description: string | null;
}

export default function ResourcesPage(): JSX.Element {
  const { user } = useAuthContext();
  const [resources, setResources] = useState<Resource[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    if (user) {
      const authUrl = typeof window !== 'undefined' 
        ? (window as any).__AUTH_URL__ || 'http://localhost:3000'
        : 'http://localhost:3000';
      
      fetch(`${authUrl}/api/personalization/resources`, {
        credentials: 'include',
      })
        .then((res) => {
          if (res.status === 401) {
            window.location.href = '/signin';
            return null;
          }
          return res.json();
        })
        .then((data) => {
          if (data) setResources(data.resources || []);
        })
        .catch(console.error)
        .finally(() => setLoading(false));
    } else {
      setLoading(false);
    }
  }, [user]);

  const handleDownload = async (resourceId: string) => {
    if (!user) {
      window.location.href = '/signin';
      return;
    }

    try {
      const authUrl = typeof window !== 'undefined' 
        ? (window as any).__AUTH_URL__ || 'http://localhost:3000'
        : 'http://localhost:3000';
      
      const response = await fetch(`${authUrl}/api/personalization/resources/${resourceId}/download`, {
        method: 'POST',
        credentials: 'include',
      });

      if (response.ok) {
        const data = await response.json();
        if (data.downloadUrl) {
          window.open(data.downloadUrl, '_blank');
        }
      } else {
        throw new Error('Download failed');
      }
    } catch (error) {
      console.error('Download error:', error);
      alert('Failed to download resource. Please try again.');
    }
  };

  if (!user) {
    return (
      <Layout title="Resources" description="Downloadable resources">
        <div className="container margin-vert--lg">
          <h1>Downloadable Resources</h1>
          <p>Please <Link to="/signin">sign in</Link> to access downloadable resources.</p>
        </div>
      </Layout>
    );
  }

  if (loading) {
    return (
      <Layout title="Resources" description="Downloadable resources">
        <div className="container margin-vert--lg">
          <div>Loading resources...</div>
        </div>
      </Layout>
    );
  }

  // Group resources by module
  const grouped = resources.reduce((acc, resource) => {
    const key = resource.moduleId || 'general';
    if (!acc[key]) acc[key] = [];
    acc[key].push(resource);
    return acc;
  }, {} as Record<string, Resource[]>);

  return (
    <Layout title="Resources" description="Downloadable resources">
      <div className="container margin-vert--lg">
        <h1>Downloadable Resources</h1>
        
        {resources.length === 0 ? (
          <p>No resources available at this time.</p>
        ) : (
          Object.entries(grouped).map(([moduleId, moduleResources]) => (
            <section key={moduleId} className={styles.resourceSection}>
              <h2>{moduleId === 'general' ? 'General Resources' : moduleId}</h2>
              <ul className={styles.resourceList}>
                {moduleResources.map((resource) => (
                  <li key={resource.id} className={styles.resourceItem}>
                    <div className={styles.resourceInfo}>
                      <h3>{resource.fileName}</h3>
                      {resource.description && <p>{resource.description}</p>}
                      <small>
                        {resource.fileType} • {resource.fileSize ? `${(resource.fileSize / 1024).toFixed(2)} KB` : 'Unknown size'}
                      </small>
                    </div>
                    <button
                      onClick={() => handleDownload(resource.id)}
                      className={styles.downloadButton}
                    >
                      Download
                    </button>
                  </li>
                ))}
              </ul>
            </section>
          ))
        )}
      </div>
    </Layout>
  );
}

