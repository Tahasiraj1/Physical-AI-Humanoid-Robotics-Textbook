# Quickstart: Enhanced Authentication UX and Personalization

**Feature**: 016-auth-ux-enhancements  
**Date**: 2025-12-08

## Overview

This guide helps developers quickly understand and start working with the enhanced authentication UX features:
- Avatar selection and display
- Improved sign-up/sign-in error handling
- Personalization questions during sign-up
- Navigation state management

## Prerequisites

- Node.js 20+ installed
- PostgreSQL database (Neon Serverless or local)
- Existing Auth service setup (from feature 015)
- Docusaurus frontend setup

## Setup Steps

### 1. Database Migration

Run the new migration to create Avatar table and extend user_profile:

```bash
cd Auth
npm run migrate:custom
```

Or manually run the migration:

```bash
cd Auth
tsx src/db/migrate-custom-tables.ts
```

**Migration File**: `Auth/src/db/migrations/002-avatar-personalization.sql`

This will:
- Create `avatar` table
- Insert 10 predefined avatars
- Add new columns to `user_profile` table
- Create necessary indexes

### 2. Add Avatar Images

Place 10 avatar image files in the frontend static directory:

```
frontend/static/avatars/
├── avatar-1.png
├── avatar-2.png
├── avatar-3.png
├── ... (10 total)
└── avatar-10.png
```

**Image Requirements**:
- Format: PNG (recommended) or JPG
- Size: 200x200px (square)
- File size: < 100KB each
- Naming: `avatar-{1-10}.png`

### 3. Backend Implementation

#### Create Avatar Service

Create `Auth/src/services/avatar-service.ts`:

```typescript
import { Pool } from 'pg';

export class AvatarService {
  constructor(private pool: Pool) {}

  async getActiveAvatars() {
    const result = await this.pool.query(
      'SELECT id, imageUrl, displayName FROM avatar WHERE isActive = true ORDER BY displayName'
    );
    return result.rows;
  }

  async getUserAvatarData(userId: string) {
    // Implementation from data-model.md query patterns
  }

  async updateUserAvatar(userId: string, avatarId: string | null) {
    // Implementation
  }
}
```

#### Create Avatar Routes

Create `Auth/src/routes/avatar.ts`:

```typescript
import { Router } from 'express';
import { requireAuth } from '../utils/auth-middleware.js';
import { AvatarService } from '../services/avatar-service.js';

const router = Router();
const avatarService = new AvatarService(pool);

// Public endpoint
router.get('/list', async (req, res) => {
  // Return all active avatars
});

// Protected endpoints
router.get('/', requireAuth, async (req, res) => {
  // Get user's avatar data
});

router.put('/', requireAuth, async (req, res) => {
  // Update user's avatar selection
});

export default router;
```

#### Update Personalization Routes

Extend `Auth/src/routes/personalization.ts` with preference update endpoints.

### 4. Frontend Implementation

#### Install Better Auth Client SDK

```bash
cd frontend
npm install better-auth
```

#### Create Auth Hook

Create `frontend/src/hooks/useAuth.ts`:

```typescript
import { createAuthClient } from 'better-auth/react';

const authClient = createAuthClient({
  baseURL: process.env.NEXT_PUBLIC_AUTH_URL || 'http://localhost:3000',
  basePath: '/api/auth',
});

export function useAuth() {
  const { data: session } = authClient.useSession();
  const user = session?.data?.user;
  
  return {
    user,
    isAuthenticated: !!user,
    signOut: () => authClient.signOut(),
  };
}
```

#### Create Avatar Component

Create `frontend/src/components/Avatar/Avatar.tsx`:

```typescript
import { useAuth } from '../../hooks/useAuth';
import { AvatarDropdown } from './AvatarDropdown';
import { FirstLetterAvatar } from './FirstLetterAvatar';

export function Avatar() {
  const { user, isAuthenticated } = useAuth();
  const [avatarData, setAvatarData] = useState(null);
  const [dropdownOpen, setDropdownOpen] = useState(false);

  // Fetch avatar data
  useEffect(() => {
    if (isAuthenticated) {
      fetch('/api/personalization/avatar')
        .then(res => res.json())
        .then(data => setAvatarData(data.avatar));
    }
  }, [isAuthenticated]);

  if (!isAuthenticated) return null;

  return (
    <div className="avatar-container">
      <button onClick={() => setDropdownOpen(!dropdownOpen)}>
        {avatarData?.hasSelectedAvatar ? (
          <img src={avatarData.selectedAvatarImageUrl} alt="Avatar" />
        ) : (
          <FirstLetterAvatar letter={avatarData?.firstLetter || '?'} />
        )}
      </button>
      {dropdownOpen && (
        <AvatarDropdown
          onClose={() => setDropdownOpen(false)}
          onOutsideClick={() => setDropdownOpen(false)}
        />
      )}
    </div>
  );
}
```

#### Update Navigation

Override Docusaurus Navbar to conditionally show buttons/avatar:

```typescript
// frontend/src/theme/Navbar/index.tsx
import { useAuth } from '../../hooks/useAuth';
import { Avatar } from '../../components/Avatar/Avatar';

export default function Navbar() {
  const { isAuthenticated } = useAuth();
  
  return (
    <nav>
      {/* ... existing nav items ... */}
      {isAuthenticated ? (
        <Avatar />
      ) : (
        <>
          <Link to="/signup">Sign Up</Link>
          <Link to="/signin">Sign In</Link>
        </>
      )}
    </nav>
  );
}
```

### 5. Update Sign-Up Form

Add avatar selection and personalization questions to sign-up form:

```typescript
// frontend/src/pages/signup.tsx
function SignUpForm() {
  const [avatarId, setAvatarId] = useState(null);
  const [softwarePrefs, setSoftwarePrefs] = useState([]);
  const [hardwarePrefs, setHardwarePrefs] = useState([]);
  const [langPrefs, setLangPrefs] = useState([]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    
    // 1. Sign up with Better Auth
    const signUpResult = await authClient.signUp.email({
      email,
      password,
      name,
    });

    if (signUpResult.error) {
      // Display error, stay on page
      setError(signUpResult.error.message);
      return;
    }

    // 2. Save avatar and preferences
    await Promise.all([
      fetch('/api/personalization/avatar', {
        method: 'PUT',
        body: JSON.stringify({ avatarId }),
      }),
      fetch('/api/personalization/preferences', {
        method: 'PUT',
        body: JSON.stringify({
          softwarePreferences: softwarePrefs,
          hardwarePreferences: hardwarePrefs,
          programmingLanguagePreferences: langPrefs,
        }),
      }),
    ]);

    // 3. Redirect to dashboard (user is already signed in)
    window.location.href = '/dashboard';
  };

  // Form JSX with avatar selection and preference questions
}
```

## Testing

### Manual Testing Checklist

1. **Avatar Display**:
   - [ ] Sign in with user who has selected avatar → avatar displays
   - [ ] Sign in with user without avatar → first-letter avatar displays
   - [ ] Click avatar → dropdown opens
   - [ ] Click avatar again → dropdown closes
   - [ ] Click outside dropdown → dropdown closes

2. **Sign-Up Flow**:
   - [ ] Sign up with new email → automatically signed in, redirected to dashboard
   - [ ] Sign up with existing email → error message, stay on sign-up page
   - [ ] Sign up with avatar selection → avatar saved
   - [ ] Sign up with preferences → preferences saved
   - [ ] Sign up without avatar/preferences → account still created

3. **Sign-In Flow**:
   - [ ] Sign in with correct credentials → redirected to dashboard
   - [ ] Sign in with wrong credentials → error message, stay on sign-in page

4. **Navigation State**:
   - [ ] When signed in → avatar visible, sign-up/sign-in buttons hidden
   - [ ] When signed out → avatar hidden, sign-up/sign-in buttons visible

5. **Dropdown Menu**:
   - [ ] Click "Profile" → navigates to profile page, dropdown closes
   - [ ] Click "Settings" → navigates to settings page, dropdown closes
   - [ ] Click "Logout" → signs out, redirects to homepage

## Common Issues

### Avatar Not Displaying

**Issue**: Avatar doesn't show after sign-in.

**Solutions**:
1. Check browser console for API errors
2. Verify avatar data endpoint returns correct format
3. Check that avatar images are in correct path (`/static/avatars/`)
4. Verify user has `selectedAvatarId` in database

### Sign-Up Not Auto-Signing In

**Issue**: User has to sign in manually after sign-up.

**Solutions**:
1. Check Better Auth configuration (should be default behavior)
2. Verify session cookie is being set
3. Check frontend is handling sign-up response correctly

### Dropdown Not Closing

**Issue**: Dropdown stays open after clicking menu item.

**Solutions**:
1. Verify `onClose` callback is called after navigation
2. Check that navigation triggers component unmount/remount
3. Add explicit state reset in navigation handlers

## Next Steps

1. Review [data-model.md](./data-model.md) for database schema details
2. Review [contracts/auth-ux-api.md](./contracts/auth-ux-api.md) for API specifications
3. Review [research.md](./research.md) for implementation patterns
4. Proceed to `/sp.tasks` to generate implementation tasks

## Resources

- Better Auth Docs: https://www.better-auth.com/docs
- Docusaurus Theme Customization: https://docusaurus.io/docs/swizzling
- React Hooks: https://react.dev/reference/react

