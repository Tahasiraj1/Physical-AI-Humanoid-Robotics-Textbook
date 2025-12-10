# Data Model: Enhanced Authentication UX and Personalization

**Feature**: 016-auth-ux-enhancements  
**Date**: 2025-12-08  
**Purpose**: Define database schema and entity relationships for avatar selection and personalization features

## Overview

This feature extends the existing authentication system with:
1. Avatar selection from predefined options
2. Personalization data collection (software, hardware, programming languages)
3. Enhanced user profile with avatar and preference data

## Entities

### Avatar

**Description**: Predefined avatar options available for user selection. System provides exactly 10 avatars.

**Table**: `avatar`

**Attributes**:
- `id` (TEXT, PRIMARY KEY): Unique identifier for the avatar (UUID format)
- `imageUrl` (TEXT, NOT NULL): URL or path to avatar image file
- `displayName` (TEXT, NULLABLE): Optional descriptive name for the avatar
- `isActive` (BOOLEAN, NOT NULL, DEFAULT true): Whether avatar is available for selection
- `createdAt` (TIMESTAMP, NOT NULL, DEFAULT NOW()): When avatar was created
- `updatedAt` (TIMESTAMP, NOT NULL, DEFAULT NOW()): When avatar was last updated

**Relationships**:
- One-to-many with User Profile (many users can select the same avatar)

**Validation Rules**:
- `imageUrl` must be a valid URL or file path
- `isActive` defaults to true
- Exactly 10 avatars must exist in the system
- `id` must be unique

**Indexes**:
- Primary key on `id`
- Index on `isActive` for filtering active avatars

**Example Data**:
```sql
INSERT INTO avatar (id, imageUrl, displayName, isActive) VALUES
('avatar-1', '/static/avatars/avatar-1.png', 'Robot Avatar 1', true),
('avatar-2', '/static/avatars/avatar-2.png', 'Robot Avatar 2', true),
-- ... 8 more avatars
```

---

### User Profile (Extended)

**Description**: Extends existing `user_profile` table from feature 015-auth-personalization with avatar selection and personalization preferences.

**Table**: `user_profile` (existing table, new columns added)

**Existing Attributes** (from feature 015):
- `userId` (TEXT, PRIMARY KEY, FOREIGN KEY to `user.id`)
- `learningLevel` (VARCHAR(50), NULLABLE)
- `preferences` (JSONB, NULLABLE)
- `createdAt` (TIMESTAMP, NOT NULL, DEFAULT NOW())
- `updatedAt` (TIMESTAMP, NOT NULL, DEFAULT NOW())

**New Attributes** (this feature):
- `selectedAvatarId` (TEXT, NULLABLE, FOREIGN KEY to `avatar.id`): Reference to selected avatar
- `softwarePreferences` (JSONB, NULLABLE): Array of selected software options
  - Example: `["ROS 2", "Gazebo", "Python"]`
- `hardwarePreferences` (JSONB, NULLABLE): Array of selected hardware options
  - Example: `["Humanoid robots", "Manipulators", "Sensors"]`
- `programmingLanguagePreferences` (JSONB, NULLABLE): Array of selected programming language options
  - Example: `["Python", "C++", "JavaScript"]`

**Relationships**:
- One-to-one with User Account (via `userId`)
- Many-to-one with Avatar (via `selectedAvatarId`, optional)

**Validation Rules**:
- `selectedAvatarId` must reference an existing avatar if provided
- `selectedAvatarId` can be NULL (user hasn't selected an avatar)
- JSONB arrays must contain only strings
- JSONB arrays can be empty arrays `[]` or NULL
- All preference fields are optional (users can skip during sign-up)

**Indexes**:
- Primary key on `userId`
- Foreign key index on `selectedAvatarId`
- GIN index on JSONB columns for efficient querying:
  - `CREATE INDEX idx_user_profile_software_prefs ON user_profile USING GIN (softwarePreferences)`
  - `CREATE INDEX idx_user_profile_hardware_prefs ON user_profile USING GIN (hardwarePreferences)`
  - `CREATE INDEX idx_user_profile_lang_prefs ON user_profile USING GIN (programmingLanguagePreferences)`

**Example Data**:
```sql
-- User with avatar and preferences
UPDATE user_profile SET
  selectedAvatarId = 'avatar-3',
  softwarePreferences = '["ROS 2", "Gazebo"]'::jsonb,
  hardwarePreferences = '["Humanoid robots", "Sensors"]'::jsonb,
  programmingLanguagePreferences = '["Python", "C++"]'::jsonb
WHERE userId = 'user-123';

-- User without avatar or preferences (new sign-up, skipped questions)
-- selectedAvatarId = NULL, all preference fields = NULL
```

---

### User Account (Better Auth)

**Description**: Existing user table managed by Better Auth. Referenced by user_profile.

**Table**: `user` (managed by Better Auth)

**Attributes** (Better Auth standard):
- `id` (TEXT, PRIMARY KEY): User identifier
- `name` (TEXT, NOT NULL): User's name (used for first-letter avatar generation)
- `email` (TEXT, NOT NULL, UNIQUE): User's email address
- `emailVerified` (BOOLEAN, NOT NULL): Email verification status
- `image` (TEXT, NULLABLE): Profile image (not used in this feature, we use avatar selection)
- `createdAt` (TIMESTAMP, NOT NULL): Account creation timestamp
- `updatedAt` (TIMESTAMP, NOT NULL): Last update timestamp

**Relationships**:
- One-to-one with User Profile

**Note**: This table is managed by Better Auth. We do not modify its structure, only reference it.

---

## Derived/Computed Entities

### Avatar Display Data

**Description**: Computed data structure for rendering user avatars in the frontend.

**Not a database table** - computed from User Profile + Avatar + User Account

**Attributes**:
- `hasSelectedAvatar` (boolean): Whether user has selected a predefined avatar
- `selectedAvatarId` (string | null): Avatar ID if selected
- `selectedAvatarImageUrl` (string | null): Avatar image URL if selected
- `userName` (string): User's name from `user` table (for first-letter extraction)
- `avatarDisplayType` ("predefined" | "letter" | "default"): Which type of avatar to display
- `firstLetter` (string | null): First letter of user's name (if using letter avatar)

**Computation Logic**:
1. If `selectedAvatarId` exists and avatar is active → `avatarDisplayType = "predefined"`
2. Else if `userName` exists and not empty → `avatarDisplayType = "letter"`, extract first letter
3. Else → `avatarDisplayType = "default"`, use generic avatar

---

## Database Migration

### Migration File: `002-avatar-personalization.sql`

**Purpose**: Add Avatar table and extend user_profile with new columns

**Changes**:
1. Create `avatar` table
2. Insert 10 predefined avatars
3. Add new columns to `user_profile` table:
   - `selectedAvatarId`
   - `softwarePreferences`
   - `hardwarePreferences`
   - `programmingLanguagePreferences`
4. Create indexes for performance
5. Add foreign key constraint for `selectedAvatarId`

**Backward Compatibility**:
- All new columns are nullable
- Existing user profiles remain valid (NULL values for new fields)
- No data migration required for existing users

**Rollback Strategy**:
- Drop new columns from `user_profile`
- Drop `avatar` table
- All changes are additive, no data loss risk

---

## Data Validation Rules

### Avatar Selection
- `selectedAvatarId` must reference an active avatar (`isActive = true`)
- If avatar is deactivated, user's `selectedAvatarId` should be set to NULL (or handled in application logic)

### Personalization Preferences
- All preference fields are arrays of strings
- Arrays can contain duplicate values (application should handle deduplication if needed)
- Maximum array size: No hard limit, but practical limit of ~50 items per preference type
- Array values should be validated against predefined option lists (application-level validation)

### First-Letter Avatar Generation
- Extract first alphanumeric character from `user.name`
- If name is empty or contains no alphanumeric characters, use default avatar
- Handle Unicode characters appropriately (extract first valid character)

---

## Query Patterns

### Get User Avatar Data
```sql
SELECT 
  u.id as userId,
  u.name,
  up.selectedAvatarId,
  a.imageUrl as avatarImageUrl,
  up.softwarePreferences,
  up.hardwarePreferences,
  up.programmingLanguagePreferences
FROM "user" u
LEFT JOIN user_profile up ON u.id = up."userId"
LEFT JOIN avatar a ON up.selectedAvatarId = a.id AND a.isActive = true
WHERE u.id = $1;
```

### Get All Active Avatars
```sql
SELECT id, imageUrl, displayName
FROM avatar
WHERE isActive = true
ORDER BY displayName;
```

### Update User Avatar Selection
```sql
UPDATE user_profile
SET selectedAvatarId = $1, updatedAt = NOW()
WHERE "userId" = $2;
```

### Update User Personalization Preferences
```sql
UPDATE user_profile
SET 
  softwarePreferences = $1::jsonb,
  hardwarePreferences = $2::jsonb,
  programmingLanguagePreferences = $3::jsonb,
  updatedAt = NOW()
WHERE "userId" = $4;
```

### Query Users by Preference (for future personalization features)
```sql
-- Find users who selected "Python" as a programming language
SELECT u.id, u.name, u.email
FROM "user" u
JOIN user_profile up ON u.id = up."userId"
WHERE up.programmingLanguagePreferences @> '["Python"]'::jsonb;
```

---

## Relationships Diagram

```
user (Better Auth)
  │
  ├─ 1:1 ──→ user_profile
  │            │
  │            ├─ selectedAvatarId (FK, nullable)
  │            │     │
  │            │     └─→ avatar (many-to-one)
  │            │
  │            ├─ softwarePreferences (JSONB array)
  │            ├─ hardwarePreferences (JSONB array)
  │            └─ programmingLanguagePreferences (JSONB array)
```

---

## Data Integrity Constraints

1. **Foreign Key**: `user_profile.selectedAvatarId` → `avatar.id`
   - ON DELETE SET NULL (if avatar is deleted, user's selection is cleared)
   - ON UPDATE CASCADE (if avatar ID changes, user selection updates)

2. **Referential Integrity**: `user_profile.userId` → `user.id`
   - ON DELETE CASCADE (if user is deleted, profile is deleted)
   - Already exists from feature 015

3. **Uniqueness**: 
   - `avatar.id` is unique (primary key)
   - `user_profile.userId` is unique (primary key, one profile per user)

4. **Check Constraints**:
   - `avatar.isActive` must be boolean
   - JSONB arrays must be valid JSON arrays of strings (application-level validation)

---

## Performance Considerations

1. **Indexes**: GIN indexes on JSONB columns enable efficient queries for preference matching
2. **Avatar Lookup**: Small table (10 rows), no performance concerns
3. **Profile Queries**: Single row per user, indexed by userId, fast lookups
4. **JSONB Queries**: GIN indexes support containment queries (`@>` operator) efficiently

---

## Future Extensibility

- Avatar table can be extended with additional metadata (categories, tags, etc.)
- Personalization preferences can be extended with new question types (add new JSONB columns)
- Preference arrays can be queried for analytics and recommendation algorithms
- Avatar selection history could be tracked (separate table) if needed for analytics

