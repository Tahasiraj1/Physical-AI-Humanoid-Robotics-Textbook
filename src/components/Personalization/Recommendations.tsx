import React, { useState, useEffect } from 'react';
import { getAuthUrl } from '@site/src/lib/auth-url';
import Link from '@docusaurus/Link';
import styles from './Recommendations.module.css';

interface Recommendation {
  moduleId: string;
  reason: string;
  priorityScore: number;
}

export default function Recommendations() {
  const [recommendations, setRecommendations] = useState<Recommendation[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const authUrl = getAuthUrl();
    
    fetch(`${authUrl}/api/personalization/recommendations`, {
      credentials: 'include',
    })
      .then((res) => res.json())
      .then((data) => setRecommendations(data.recommendations || []))
      .catch(console.error)
      .finally(() => setLoading(false));
  }, []);

  if (loading) return <div>Loading recommendations...</div>;
  if (recommendations.length === 0) return <div>No recommendations available.</div>;

  return (
    <div className={styles.recommendations}>
      <h3>Recommended Next Steps</h3>
      <ul className={styles.recommendationsList}>
        {recommendations.map((rec) => (
          <li key={rec.moduleId} className={styles.recommendationItem}>
            <Link to={`/modules/${rec.moduleId}/`} className={styles.recommendationLink}>
              {rec.moduleId}
            </Link>
            <span className={styles.reason}>{rec.reason}</span>
          </li>
        ))}
      </ul>
    </div>
  );
}

