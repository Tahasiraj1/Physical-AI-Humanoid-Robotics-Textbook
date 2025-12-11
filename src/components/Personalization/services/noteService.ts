import { getAuthUrl } from '@site/src/lib/auth-url';

const API_BASE = '/api/personalization';

export class NoteService {
  private getAuthUrl(): string {
    return getAuthUrl(); // Use shared utility
  }

  async createOrUpdateNote(moduleId: string, sectionId: string, content: string) {
    const authUrl = this.getAuthUrl();
    const response = await fetch(`${authUrl}${API_BASE}/notes`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify({ moduleId, sectionId, content }),
    });
    if (!response.ok) throw new Error('Failed to save note');
    return response.json();
  }

  async getNotes(moduleId?: string, sectionId?: string) {
    const authUrl = this.getAuthUrl();
    let url = `${authUrl}${API_BASE}/notes`;
    const params = new URLSearchParams();
    if (moduleId) params.append('moduleId', moduleId);
    if (sectionId) params.append('sectionId', sectionId);
    if (params.toString()) url += `?${params.toString()}`;
    
    const response = await fetch(url, { credentials: 'include' });
    if (!response.ok) throw new Error('Failed to fetch notes');
    return response.json();
  }

  async deleteNote(id: string) {
    const authUrl = this.getAuthUrl();
    const response = await fetch(`${authUrl}${API_BASE}/notes/${id}`, {
      method: 'DELETE',
      credentials: 'include',
    });
    if (!response.ok) throw new Error('Failed to delete note');
    return response.json();
  }
}

export const noteService = new NoteService();

