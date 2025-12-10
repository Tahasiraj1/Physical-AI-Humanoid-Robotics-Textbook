import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const API_BASE = '/api/personalization';

/**
 * Progress tracking service with functions to record and fetch progress
 */
export class ProgressService {
  private getAuthUrl(): string {
    try {
      const { siteConfig } = useDocusaurusContext();
      return (siteConfig.customFields?.authUrl as string) || 'http://localhost:3000';
    } catch {
      return typeof window !== 'undefined' 
        ? (window as any).__AUTH_URL__ || 'http://localhost:3000'
        : 'http://localhost:3000';
    }
  }

  /**
   * Record a section view
   */
  async recordSectionView(moduleId: string, sectionId: string): Promise<void> {
    const authUrl = this.getAuthUrl();
    const response = await fetch(`${authUrl}${API_BASE}/progress`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include',
      body: JSON.stringify({ moduleId, sectionId }),
    });

    if (!response.ok) {
      throw new Error(`Failed to record progress: ${response.statusText}`);
    }
  }

  /**
   * Get user progress
   */
  async getUserProgress(moduleId?: string): Promise<any> {
    const authUrl = this.getAuthUrl();
    const url = moduleId 
      ? `${authUrl}${API_BASE}/progress?moduleId=${moduleId}`
      : `${authUrl}${API_BASE}/progress`;
    
    const response = await fetch(url, {
      method: 'GET',
      credentials: 'include',
    });

    if (!response.ok) {
      throw new Error(`Failed to fetch progress: ${response.statusText}`);
    }

    return response.json();
  }
}

export const progressService = new ProgressService();

