const API_BASE = '/api/personalization';

export class BookmarkService {
  private getAuthUrl(): string {
    // Get auth URL from window or default
    return typeof window !== 'undefined' 
      ? (window as any).__AUTH_URL__ || 
        (document.querySelector('meta[name="auth-url"]')?.getAttribute('content')) ||
        'http://localhost:3000'
      : 'http://localhost:3000';
  }

  async createBookmark(moduleId: string, sectionId: string) {
    const authUrl = this.getAuthUrl();
    const response = await fetch(`${authUrl}${API_BASE}/bookmarks`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify({ moduleId, sectionId }),
    });
    
    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      const error = new Error(errorData.message || 'Failed to create bookmark');
      (error as any).response = response;
      (error as any).status = response.status;
      throw error;
    }
    
    return response.json();
  }

  async deleteBookmark(id: string) {
    const authUrl = this.getAuthUrl();
    const response = await fetch(`${authUrl}${API_BASE}/bookmarks/${id}`, {
      method: 'DELETE',
      credentials: 'include',
    });
    
    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      const error = new Error(errorData.message || 'Failed to delete bookmark');
      (error as any).response = response;
      (error as any).status = response.status;
      throw error;
    }
    
    return response.json();
  }

  async getBookmarks(moduleId?: string) {
    const authUrl = this.getAuthUrl();
    const url = moduleId 
      ? `${authUrl}${API_BASE}/bookmarks?moduleId=${moduleId}`
      : `${authUrl}${API_BASE}/bookmarks`;
    const response = await fetch(url, { credentials: 'include' });
    if (!response.ok) throw new Error('Failed to fetch bookmarks');
    return response.json();
  }

  async checkBookmark(moduleId: string, sectionId: string) {
    const authUrl = this.getAuthUrl();
    const response = await fetch(
      `${authUrl}${API_BASE}/bookmarks/check?moduleId=${encodeURIComponent(moduleId)}&sectionId=${encodeURIComponent(sectionId)}`,
      { credentials: 'include' }
    );
    
    if (!response.ok) {
      // If 401, user is not authenticated - this is OK, just return not bookmarked
      if (response.status === 401) {
        return { isBookmarked: false, bookmark: null };
      }
      const errorData = await response.json().catch(() => ({}));
      const error = new Error(errorData.message || 'Failed to check bookmark');
      (error as any).response = response;
      (error as any).status = response.status;
      throw error;
    }
    
    return response.json();
  }
}

export const bookmarkService = new BookmarkService();

