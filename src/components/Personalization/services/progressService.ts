import { getAuthUrl } from '@site/src/lib/auth-url';

const API_BASE = '/api/personalization';

/**
 * Progress tracking service with functions to record and fetch progress
 */
export class ProgressService {
  private getAuthUrl(): string {
    return getAuthUrl(); // Use shared utility
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

