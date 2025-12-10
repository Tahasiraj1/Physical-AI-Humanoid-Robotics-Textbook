import React, { useState, useEffect } from 'react';
import { noteService } from './services/noteService';
import Link from '@docusaurus/Link';
import styles from './NotesList.module.css';

interface Note {
  id: string;
  moduleId: string;
  sectionId: string;
  content: string;
  updatedAt: string;
}

export default function NotesList() {
  const [notes, setNotes] = useState<Note[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    noteService.getNotes()
      .then((data) => setNotes(data.notes || []))
      .catch(console.error)
      .finally(() => setLoading(false));
  }, []);

  if (loading) return <div>Loading notes...</div>;
  if (notes.length === 0) return <div>No notes yet.</div>;

  return (
    <div className={styles.notesList}>
      {notes.map((note) => (
        <div key={note.id} className={styles.noteItem}>
          <div className={styles.noteHeader}>
            <Link to={`/modules/${note.moduleId}/${note.sectionId}`} className={styles.noteLink}>
              {note.moduleId} / {note.sectionId}
            </Link>
            <span className={styles.noteDate}>
              {new Date(note.updatedAt).toLocaleDateString()}
            </span>
          </div>
          <div className={styles.notePreview}>
            {note.content.substring(0, 100)}
            {note.content.length > 100 ? '...' : ''}
          </div>
        </div>
      ))}
    </div>
  );
}

