# Feature Specification: Enhanced Authentication UX and Personalization

**Feature Branch**: `016-auth-ux-enhancements`  
**Created**: 2025-12-08  
**Status**: Draft  
**Input**: User description: "When the user is signed-in - He should see an Avatar with profile pic from db (if available, if not make an Avatar with his name's first letter). If an account already exists in db show user, a message about it, and don't redirect user to dashboard keep him on sign-up unless he navigates manually. If sign-in fails show message and keep him on sign-in page unless he navigates manually. Update sign-up form with 3 questions (updates db schema and migrate): Which softwares do you use? Which hardwares do you use or are interested in. Make another one like the two above. These questions will be used for personalization."

## Clarifications

### Session 2025-12-08

- Q: What should the third personalization question be? → A: "Which programming languages do you use?" (similar format to software/hardware questions, enables content personalization for robotics textbook audience)
- Q: How should profile pictures be stored? → A: Users select from 10 predefined avatars provided by the system. Avatars are stored in an Avatar model/table, and user profiles reference avatar IDs. This eliminates file uploads, file size validation, and storage concerns.
- Q: What format should personalization questions use? → A: Predefined multi-select options (users select from fixed lists via checkboxes). This ensures consistent data for personalization algorithms, simplifies validation, and provides better UX.
- Q: When should personalization questions appear in the sign-up flow? → A: During account creation (all fields including email, password, and personalization questions on one form, submitted together). This creates a single-step sign-up process.
- Q: When can users select avatars? → A: Both (optional during sign-up on the same form, can change later in profile settings). This provides flexibility without blocking account creation.
- Q: Should users be automatically signed in after successful sign-up? → A: Yes, users are automatically signed in after successful sign-up (session created immediately). Users do not need to sign in again after creating an account.
- Q: How should navigation UI change based on authentication state? → A: Hide sign-up/sign-in buttons when signed in; show avatar with dropdown menu containing logout, profile, and settings options. This provides clear visual indication of authentication state and easy access to user actions.
- Q: What should be in the avatar dropdown menu and in what order? → A: Standard menu with Profile, Settings, Logout (in that order). Profile navigates to `/profile` page, Settings navigates to `/profile` page and shows settings section (scrolls to or displays settings section), Logout signs user out and redirects to homepage. Menu closes automatically after selection.
- Q: Should profile and settings be separate routes? → A: No, profile and settings are on one route `/profile` with a settings section underneath the profile content. Clicking "Settings" from dropdown navigates to `/profile` and displays/shows the settings section.
- Q: How should the avatar dropdown interact with clicks? → A: Toggle dropdown on avatar click (opens if closed, closes if open); close dropdown when clicking outside; avatar itself doesn't navigate (only dropdown items navigate). This follows standard dropdown interaction patterns.
- Q: Where should users be redirected after successful sign-up? → A: Always redirect to dashboard after successful sign-up. This provides a clear destination where new users can see their personalized content, progress, and recommendations.
- Q: Should profile and settings be separate routes? → A: No, profile and settings are on one route `/profile` with a settings section underneath the profile content.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Avatar Display and Navigation for Signed-In Users (Priority: P1)

A signed-in user navigates through the textbook website. They see their avatar displayed in the navigation bar or header, and sign-up/sign-in buttons are hidden. If they have selected a predefined avatar from the system's collection, that avatar is displayed. If they have not selected an avatar, an avatar is automatically generated showing the first letter of their name. Clicking on the avatar opens a dropdown menu with options such as logout, profile, and settings. The avatar is visible on all pages while they remain signed in.

**Why this priority**: Avatar display provides immediate visual feedback that the user is authenticated and personalizes their experience. This is a foundational UX enhancement that improves user recognition and engagement.

**Independent Test**: Can be fully tested by signing in with an account that has a profile picture and verifying the picture displays, then signing in with an account without a profile picture and verifying the first-letter avatar displays. This delivers value by providing clear visual authentication status and personalization.

**Acceptance Scenarios**:

1. **Given** a user has signed in and has selected a predefined avatar (during sign-up or later in profile), **When** they navigate to any page, **Then** their selected avatar is displayed in the navigation/header area and sign-up/sign-in buttons are hidden
2. **Given** a user has signed in and has not selected a predefined avatar, **When** they navigate to any page, **Then** an avatar displaying the first letter of their name is shown in the navigation/header area and sign-up/sign-in buttons are hidden
3. **Given** a user's name is "John Doe", **When** they sign in without a selected avatar, **Then** an avatar with the letter "J" is displayed
4. **Given** a user has a single-word name "Alice", **When** they sign in without a selected avatar, **Then** an avatar with the letter "A" is displayed
5. **Given** a signed-in user clicks on their avatar when the dropdown is closed, **When** the avatar is clicked, **Then** a dropdown menu appears with options: Profile, Settings, Logout (in that order)
6. **Given** a signed-in user clicks on their avatar when the dropdown is open, **When** the avatar is clicked again, **Then** the dropdown closes
7. **Given** a signed-in user has the avatar dropdown open, **When** they click outside the dropdown area, **Then** the dropdown closes
8. **Given** a signed-in user clicks "Profile" from the avatar dropdown, **When** they click Profile, **Then** they are taken to `/profile` page and the dropdown closes
9. **Given** a signed-in user clicks "Settings" from the avatar dropdown, **When** they click Settings, **Then** they are taken to `/profile` page and the settings section is displayed (scrolled to or shown), and the dropdown closes
10. **Given** a signed-in user clicks "Logout" from the avatar dropdown, **When** they click Logout, **Then** they are signed out, redirected to the homepage, the avatar disappears, and sign-up/sign-in buttons become visible again
11. **Given** a signed-in user clicks on the avatar itself (not a dropdown item), **When** they click, **Then** the dropdown toggles but no navigation occurs (avatar is not a link)
9. **Given** a user signs out, **When** they navigate to any page, **Then** the avatar is no longer displayed and sign-up/sign-in buttons are visible
10. **Given** a user selects a different predefined avatar from their profile settings, **When** they navigate to any page, **Then** the new selected avatar is displayed instead of the letter avatar
11. **Given** a user is on the sign-up form, **When** they view the form, **Then** they can optionally select a predefined avatar (avatar selection is not required)
12. **Given** a user completes sign-up without selecting an avatar, **When** they later visit their profile settings, **Then** they can select a predefined avatar
13. **Given** an unauthenticated user visits any page, **When** they view the navigation/header, **Then** sign-up and sign-in buttons are visible and no avatar is displayed
14. **Given** a signed-in user opens the avatar dropdown, **When** they click on any menu item, **Then** the dropdown closes automatically after the action

---

### User Story 2 - Improved Sign-Up Error Handling (Priority: P1)

A new user attempts to create an account with an email address that already exists in the database. Instead of being automatically redirected, they remain on the sign-up page and see a clear error message indicating that an account with that email already exists. They can then choose to navigate to the sign-in page manually or try a different email address.

**Why this priority**: Preventing automatic redirects on sign-up errors improves user control and reduces confusion. Users can correct their input or choose to sign in instead, enhancing the overall authentication experience.

**Independent Test**: Can be fully tested by attempting to sign up with an existing email address and verifying that an error message appears on the sign-up page without automatic redirection. This delivers value by providing clear feedback and maintaining user context.

**Acceptance Scenarios**:

1. **Given** a user is on the sign-up page, **When** they enter an email address that already exists in the database and submit the form, **Then** they see an error message indicating the account already exists and remain on the sign-up page
2. **Given** a user sees the "account already exists" error message, **When** they click a link to navigate to the sign-in page, **Then** they are taken to the sign-in page
3. **Given** a user sees the "account already exists" error message, **When** they change the email address and submit again, **Then** the form processes the new email address
4. **Given** a user successfully creates an account with a new email, **When** they submit the sign-up form, **Then** they are automatically signed in (session created) and redirected to the dashboard
5. **Given** a user attempts to sign up with an existing email, **When** the error message appears, **Then** the error message is clear and actionable (e.g., "An account with this email already exists. Please sign in or use a different email.")

---

### User Story 3 - Improved Sign-In Error Handling (Priority: P1)

A user attempts to sign in with incorrect credentials (wrong email or password). Instead of being automatically redirected, they remain on the sign-in page and see a clear error message indicating that the sign-in failed. They can then correct their credentials and try again, or navigate away manually if needed.

**Why this priority**: Keeping users on the sign-in page when authentication fails allows them to immediately correct their credentials without losing context. This reduces friction and improves the authentication experience.

**Independent Test**: Can be fully tested by attempting to sign in with incorrect credentials and verifying that an error message appears on the sign-in page without automatic redirection. This delivers value by providing immediate feedback and maintaining user context.

**Acceptance Scenarios**:

1. **Given** a user is on the sign-in page, **When** they enter incorrect email or password and submit the form, **Then** they see an error message indicating sign-in failed and remain on the sign-in page
2. **Given** a user sees a sign-in error message, **When** they correct their credentials and submit again, **Then** the form processes the new credentials
3. **Given** a user successfully signs in with correct credentials, **When** they submit the sign-in form, **Then** they are redirected to the dashboard
4. **Given** a user attempts to sign in with incorrect credentials, **When** the error message appears, **Then** the error message is clear and does not reveal whether the email exists or the password is wrong (security best practice)
5. **Given** a user sees a sign-in error message, **When** they click a "Forgot Password" link, **Then** they are taken to the password reset page
6. **Given** a user sees a sign-in error message, **When** they click a link to navigate to the sign-up page, **Then** they are taken to the sign-up page

---

### User Story 4 - Enhanced Sign-Up Form with Personalization Questions (Priority: P2)

A new user creates an account and is presented with three personalization questions during the sign-up process: "Which softwares do you use?", "Which hardwares do you use or are interested in?", and "Which programming languages do you use?". Their responses are saved to their user profile and will be used for personalization features such as recommendations and content filtering.

**Why this priority**: Collecting personalization data during sign-up enables future personalized experiences, but this feature depends on the core authentication flow being stable first. This enhances the value proposition of account creation.

**Independent Test**: Can be fully tested by completing the sign-up form with all three questions, verifying responses are saved to the database, and confirming the data is accessible for personalization features. This delivers value by enabling personalized content recommendations and filtering.

**Acceptance Scenarios**:

1. **Given** a user is on the sign-up page, **When** they view the sign-up form, **Then** they see email, password fields, and three personalization questions all on the same form
2. **Given** a user is answering the personalization questions, **When** they select multiple software options from predefined checkboxes (e.g., "ROS 2", "Gazebo", "Python"), **Then** their selections are saved to their profile
3. **Given** a user is answering the personalization questions, **When** they select multiple hardware options from predefined checkboxes (e.g., "Humanoid robots", "Manipulators", "Sensors"), **Then** their selections are saved to their profile
4. **Given** a user is answering the personalization questions, **When** they select multiple programming language options from predefined checkboxes (e.g., "Python", "C++", "JavaScript"), **Then** their selections are saved to their profile
5. **Given** a user completes the sign-up form with all questions, **When** they submit the form, **Then** their account is created, they are automatically signed in, and all personalization data is saved
6. **Given** a user has completed sign-up with personalization questions, **When** they view their profile later, **Then** they can see and edit their personalization preferences
7. **Given** a user skips or leaves personalization questions blank during sign-up, **When** they submit the form, **Then** their account is still created successfully with empty/null personalization data

---

### Edge Cases

- What happens when a user's name is empty or contains only spaces? The system should use a default avatar (e.g., "?" or a generic user icon) or extract the first letter from the email address
- How does the system handle special characters in names for avatar generation? The system should extract the first alphanumeric character or use a default avatar
- What happens when a user selects an avatar that is later removed from the system? The system should fall back to the first-letter avatar or assign a default avatar
- How does the system handle concurrent sign-up attempts with the same email? The system should show the "account already exists" error for the second attempt
- What happens when sign-in fails due to network issues? The system should show a network error message and allow retry without redirecting
- How does the system handle users who select all available options for a personalization question? The system should allow all selections (no limit on number of selections per question)
- What happens when a user selects no options for personalization questions? The system should save empty/null values and allow users to update later
- What happens when a user changes their name after account creation? The avatar should update to show the new first letter if no predefined avatar is selected
- How does the system handle users with non-ASCII characters in their names? The system should extract the first character appropriately or use a default avatar
- What happens when a user has not selected any predefined avatar? The system should display the first-letter avatar
- What happens when an unauthenticated user visits the site? Sign-up and sign-in buttons are visible, no avatar is displayed
- What happens when a user clicks outside the avatar dropdown menu? The dropdown closes automatically
- What happens when a user navigates to a new page while the dropdown is open? The dropdown should close automatically
- How does the system handle clicking on the avatar? The dropdown toggles (opens if closed, closes if open) - standard toggle behavior
- What happens when a user clicks the avatar itself (not a dropdown item)? The dropdown toggles but no navigation occurs (avatar is not a link, only dropdown items navigate)
- What happens after a user clicks "Logout" from the dropdown? User is signed out, session is destroyed, redirected to homepage, and navigation shows sign-up/sign-in buttons
- What happens after a user clicks "Profile" from the dropdown? User is navigated to `/profile` page and dropdown closes
- What happens after a user clicks "Settings" from the dropdown? User is navigated to `/profile` page with settings section displayed (scrolled to or shown) and dropdown closes

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display an avatar for all signed-in users in the navigation/header area
- **FR-002**: System MUST display the user's selected predefined avatar if one is chosen in their account
- **FR-003**: System MUST generate and display an avatar with the first letter of the user's name if no predefined avatar is selected
- **FR-004**: System MUST extract the first letter from the user's name for avatar generation (using the first word if name contains multiple words)
- **FR-005**: System MUST handle cases where the user's name is empty or invalid by displaying a default avatar
- **FR-006**: System MUST provide a collection of 10 predefined avatars for users to select from
- **FR-007**: System MUST store avatar selections as references (avatar IDs) in the user profile database
- **FR-008**: System MUST allow users to view and change their selected avatar from their profile settings
- **FR-032**: System MUST hide sign-up and sign-in buttons when a user is signed in
- **FR-033**: System MUST show sign-up and sign-in buttons when a user is not signed in
- **FR-034**: System MUST make the avatar clickable and toggle a dropdown menu when clicked (opens if closed, closes if open)
- **FR-035**: System MUST close the avatar dropdown menu when user clicks outside the dropdown area
- **FR-036**: System MUST NOT make the avatar itself a navigation link (avatar click only toggles dropdown, doesn't navigate)
- **FR-037**: System MUST include "Profile" option in the avatar dropdown menu (first item) that navigates to user's profile page at `/profile`
- **FR-038**: System MUST include "Settings" option in the avatar dropdown menu (second item) that navigates to the settings section on the same `/profile` page (scrolls to or shows settings section)
- **FR-039**: System MUST include "Logout" option in the avatar dropdown menu (third item) that signs user out and redirects to homepage
- **FR-040**: System MUST close the avatar dropdown menu automatically after user selects any menu item
- **FR-041**: System MUST order dropdown menu items as: Profile, Settings, Logout (in that sequence)
- **FR-009**: System MUST display error messages on the sign-up page when an account with the provided email already exists
- **FR-010**: System MUST keep users on the sign-up page when account creation fails due to existing email (no automatic redirect)
- **FR-011**: System MUST allow users to manually navigate away from the sign-up page after seeing an error message
- **FR-012**: System MUST display error messages on the sign-in page when authentication fails
- **FR-013**: System MUST keep users on the sign-in page when sign-in fails (no automatic redirect)
- **FR-014**: System MUST allow users to manually navigate away from the sign-in page after seeing an error message
- **FR-015**: System MUST provide clear, actionable error messages for both sign-up and sign-in failures
- **FR-016**: System MUST include avatar selection (optional) and three personalization questions on the same sign-up form as email and password fields (all fields submitted together)
- **FR-017**: System MUST include three personalization questions in the sign-up form: software preferences, hardware preferences, and programming language preferences
- **FR-018**: System MUST present personalization questions as predefined multi-select checkboxes (users select from fixed option lists)
- **FR-019**: System MUST provide predefined option lists for each personalization question (software list, hardware list, programming language list)
- **FR-020**: System MUST save user responses to personalization questions to the database during account creation (in the same transaction as account creation)
- **FR-030**: System MUST automatically sign in users after successful account creation (create authentication session immediately)
- **FR-031**: System MUST NOT require users to manually sign in after completing sign-up
- **FR-042**: System MUST redirect users to the dashboard after successful sign-up
- **FR-043**: System MUST redirect users to the dashboard after successful sign-in
- **FR-021**: System MUST store personalization data in the user profile or a related database table
- **FR-022**: System MUST allow users to view and edit their personalization preferences after account creation
- **FR-023**: System MUST use personalization data for content recommendations and filtering *(Note: This requirement is deferred to a future feature for personalized content recommendations and filtering. Personalization data collected in this feature will be used by the future recommendation system.)*
- **FR-024**: System MUST handle cases where users skip or leave personalization questions blank (no selections made) - questions are optional
- **FR-025**: System MUST validate that selected options exist in the predefined lists (prevent invalid selections)
- **FR-026**: System MUST update the database schema to support avatar selection and personalization questions
- **FR-027**: System MUST provide database migration scripts for schema updates
- **FR-028**: System MUST create an Avatar model/table to store the 10 predefined avatars
- **FR-029**: System MUST ensure all 10 predefined avatars are available for user selection

### Key Entities *(include if feature involves data)*

- **Avatar**: Represents a predefined avatar option available for user selection. Attributes: avatarId (unique identifier), avatarImageUrl (URL or path to avatar image), displayName (optional descriptive name), isActive (boolean indicating if avatar is available). Relationships: can be selected by multiple User Profiles (one-to-many)

- **User Profile**: Extends the existing user account with additional attributes. New attributes: selectedAvatarId (optional foreign key reference to Avatar), softwarePreferences (array or JSON field storing selected software options), hardwarePreferences (array or JSON field storing selected hardware options), programmingLanguagePreferences (array or JSON field storing selected programming language options). Relationships: belongs to User Account (one-to-one), optionally references Avatar (many-to-one)

- **Avatar Display Data**: Represents the data needed to render a user avatar. Attributes: hasSelectedAvatar (boolean), selectedAvatarId (optional reference), selectedAvatarImageUrl (optional string), userName (string for first letter extraction), avatarDisplayType ("predefined" | "letter" | "default"). Relationships: derived from User Profile and Avatar entities

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of signed-in users see an avatar (either selected predefined avatar or first-letter avatar) within 1 second of page load
- **SC-002**: Avatar displays correctly for 99% of users (predefined avatar loads or first-letter avatar generates without errors)
- **SC-003**: When sign-up fails due to existing email, 100% of users see an error message and remain on the sign-up page (no automatic redirects)
- **SC-004**: When sign-in fails, 100% of users see an error message and remain on the sign-in page (no automatic redirects)
- **SC-005**: Error messages for sign-up and sign-in failures are displayed within 2 seconds of form submission
- **SC-006**: 95% of users who see sign-up/sign-in error messages can successfully correct their input and complete authentication on the next attempt
- **SC-007**: 90% of new users complete all three personalization questions during sign-up
- **SC-008**: Personalization question responses are saved to the database with 100% reliability during account creation
- **SC-009**: Users can view and edit their personalization preferences within 3 seconds of accessing their profile page
- **SC-010**: Database schema migration completes successfully without data loss for existing user accounts
- **SC-011**: Avatar selection updates complete successfully 100% of the time (avatar ID reference updates immediately)
- **SC-012**: First-letter avatar generation works correctly for 99% of user names (handles edge cases like empty names, special characters)

## Assumptions

- Users have JavaScript enabled in their browsers (required for React/Docusaurus)
- The system provides exactly 10 predefined avatars for user selection (stored in Avatar model/table)
- Users select avatars by referencing avatar IDs (no file uploads required)
- The third personalization question is "Which programming languages do you use?" and follows the same format as software/hardware questions (predefined multi-select checkboxes)
- Personalization questions use predefined option lists with multi-select checkboxes (users can select multiple options from fixed lists)
- Predefined option lists for software, hardware, and programming languages must be curated and maintained by the system
- Personalization questions appear on the same sign-up form as email and password fields (all submitted together in a single form submission)
- Avatar selection is optional during sign-up (users can select an avatar on the sign-up form or skip and select later in profile settings)
- Users can change their selected avatar at any time through profile settings
- Users can update their selected avatar and personalization preferences after account creation
- The avatar component will be integrated into the existing navigation/header structure
- Error messages will be displayed inline on the respective forms (sign-up or sign-in page)
- Database migration will be backward compatible (existing users without new fields will have null/empty values)
- Predefined avatars are stored as image files (URLs or paths) and referenced by ID
- The first letter extraction will use the first character of the first word of the user's name
- Better Auth library supports custom error handling and prevents automatic redirects on authentication failures
- Personalization data will be used by recommendation algorithms and content filtering features (implementation details in future features)

## Dependencies

- Existing authentication system (Better Auth) must be functional and accessible
- User profile database table must exist (from feature 015-auth-personalization)
- Database migration framework must be in place
- Frontend avatar component library or custom avatar implementation must be available
- Avatar image files (10 predefined avatars) must be created and stored (as static assets or in storage service)
- Navigation/header component must support avatar integration

## Out of Scope

- Advanced avatar customization (background colors, shapes, borders)
- User-uploaded profile pictures or picture galleries
- Automatic avatar generation from external services (Gravatar, etc.)
- Real-time avatar updates across multiple browser tabs
- Avatar cropping or editing tools
- Adding more than 10 predefined avatars (initial set is fixed at 10)
- Social authentication integration (Google, GitHub, etc.) - covered in feature 015
- Two-factor authentication - covered in feature 015
- Password strength meter or advanced password validation
- Email verification workflow changes
- Advanced personalization question types (free text, file uploads, etc.)
- Personalization question branching logic (conditional questions)
- Export of personalization data
- Analytics or tracking of personalization question completion rates
