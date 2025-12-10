import React, { useState, useEffect } from 'react';
import { useAuthContext } from '@site/src/components/Auth/AuthProvider';
import { noteService } from './services/noteService';
import Link from '@docusaurus/Link';
import styles from './NoteEditor.module.css';

interface NoteEditorProps {
  moduleId: string;
  sectionId: string;
}

export default function NoteEditor({ moduleId, sectionId }: NoteEditorProps) {
  const { user } = useAuthContext();
  const [content, setContent] = useState('');
  const [noteId, setNoteId] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const [saving, setSaving] = useState(false);

  useEffect(() => {
    if (user) {
      setLoading(true);
      noteService.getNotes(moduleId, sectionId)
        .then((data) => {
          const note = data.notes?.[0];
          if (note) {
            setContent(note.content);
            setNoteId(note.id);
          }
        })
        .catch(console.error)
        .finally(() => setLoading(false));
    }
  }, [user, moduleId, sectionId]);

  const handleSave = async () => {
    if (!user) {
      const signIn = confirm('Please sign in to save notes. Would you like to sign in now?');
      if (signIn) {
        window.location.href = `/signin?returnUrl=${encodeURIComponent(window.location.pathname)}`;
      }
      return;
    }

    setSaving(true);
    try {
      await noteService.createOrUpdateNote(moduleId, sectionId, content);
      alert('Note saved successfully!');
    } catch (error) {
      console.error('Note save error:', error);
      alert('Failed to save note. Please try again.');
    } finally {
      setSaving(false);
    }
  };

  const handleDelete = async () => {
    if (!noteId) return;
    if (!confirm('Are you sure you want to delete this note?')) return;

    try {
      await noteService.deleteNote(noteId);
      setContent('');
      setNoteId(null);
      alert('Note deleted successfully!');
    } catch (error) {
      console.error('Note delete error:', error);
      alert('Failed to delete note. Please try again.');
    }
  };

  if (!user) {
    return (
      <div className={styles.noteEditor}>
        <p>Please <Link to="/signin">sign in</Link> to add notes.</p>
      </div>
    );
  }

  if (loading) return <div>Loading note...</div>;

  return (
    <div className={styles.noteEditor}>
      <h3>Your Note</h3>
      <textarea
        value={content}
        onChange={(e) => setContent(e.target.value)}
        placeholder="Add your personal notes about this section..."
        className={styles.textarea}
        rows={6}
      />
      <div className={styles.actions}>
        <button onClick={handleSave} disabled={saving} className={styles.saveButton}>
          {saving ? 'Saving...' : 'Save Note'}
        </button>
        {noteId && (
          <button onClick={handleDelete} className={styles.deleteButton}>
            Delete Note
          </button>
        )}
      </div>
    </div>
  );
}

