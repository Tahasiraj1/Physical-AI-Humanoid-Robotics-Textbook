# Quickstart Guide: User Authentication and Content Personalization

**Feature**: 015-auth-personalization  
**Date**: 2025-01-27

## Overview

This guide provides step-by-step instructions for setting up the authentication and personalization features locally for development.

## Prerequisites

- Node.js >= 20.0
- PostgreSQL database (Neon Serverless Postgres recommended)
- Git
- npm or pnpm

## Step 1: Database Setup

### Create Neon Postgres Database

1. Sign up for [Neon](https://neon.tech) (free tier available)
2. Create a new project
3. Copy the connection string (format: `postgresql://user:password@host/database`)

### Set Environment Variable

```bash
export DATABASE_URL="postgresql://user:password@host/database"
```

## Step 2: Auth Service Setup

### Create Auth Folder Structure

```bash
mkdir -p Auth/src
cd Auth
```

### Initialize Node.js Project

```bash
npm init -y
```

### Install Dependencies

```bash
npm install express better-auth cors dotenv
npm install -D @types/express @types/cors @types/node typescript ts-node
```

### Create TypeScript Configuration

Create `tsconfig.json`:

```json
{
  "compilerOptions": {
    "target": "ES2020",
    "module": "commonjs",
    "lib": ["ES2020"],
    "outDir": "./dist",
    "rootDir": "./src",
    "strict": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "forceConsistentCasingInFileNames": true,
    "resolveJsonModule": true
  },
  "include": ["src/**/*"],
  "exclude": ["node_modules", "dist"]
}
```

### Create Environment File

Create `Auth/.env`:

```env
DATABASE_URL=postgresql://user:password@host/database
BETTER_AUTH_SECRET=your-random-secret-here
BETTER_AUTH_URL=http://localhost:3000
FRONTEND_URL=http://localhost:3001
EMAIL_SERVER=smtp://user:pass@smtp.example.com
```

Generate a random secret:
```bash
node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"
```

### Create Auth Configuration

Create `Auth/src/auth.ts`:

```typescript
import { betterAuth } from "better-auth"
import { prismaAdapter } from "better-auth/adapters/prisma"
import { prisma } from "./db"

export const auth = betterAuth({
  database: prismaAdapter(prisma, {
    provider: "postgresql"
  }),
  emailAndPassword: {
    enabled: true
  },
  baseURL: process.env.BETTER_AUTH_URL,
  secret: process.env.BETTER_AUTH_SECRET,
})
```

### Create Express Server

Create `Auth/src/index.ts`:

```typescript
import express from "express"
import cors from "cors"
import { toNodeHandler } from "better-auth/node"
import { auth } from "./auth"
import dotenv from "dotenv"

dotenv.config()

const app = express()
const port = process.env.PORT || 3000

// CORS configuration
app.use(cors({
  origin: process.env.FRONTEND_URL || "http://localhost:3001",
  methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
  credentials: true,
  allowedHeaders: ["Content-Type", "Authorization"]
}))

// Better Auth handler
app.all("/api/auth/*", toNodeHandler(auth))

// Health check
app.get("/health", (req, res) => {
  res.json({ status: "ok" })
})

app.listen(port, () => {
  console.log(`Auth service running on http://localhost:${port}`)
})
```

### Run Database Migrations

```bash
cd Auth
npx @better-auth/cli migrate
```

This creates Better Auth tables in your database.

### Create Custom Tables

Run SQL migrations for custom personalization tables (see `data-model.md` for schema).

### Start Auth Service

```bash
npm run dev
```

Or add to `package.json`:
```json
{
  "scripts": {
    "dev": "ts-node src/index.ts",
    "build": "tsc",
    "start": "node dist/index.js"
  }
}
```

## Step 3: Docusaurus Frontend Setup

### Install Dependencies

In the repository root:

```bash
npm install better-auth
```

### Create Auth Client

Create `src/lib/auth-client.ts`:

```typescript
import { createAuthClient } from "better-auth/react"

export const authClient = createAuthClient({
  baseURL: process.env.NEXT_PUBLIC_AUTH_URL || "http://localhost:3000"
})
```

### Create Environment Variable

Create `.env.local` (or add to existing):

```env
NEXT_PUBLIC_AUTH_URL=http://localhost:3000
```

### Create Sign-In Page

Create `src/pages/signin.tsx`:

```typescript
import Layout from "@theme/Layout"
import { SignIn } from "../components/Auth/SignIn"

export default function SignInPage() {
  return (
    <Layout title="Sign In">
      <div className="container margin-vert--lg">
        <SignIn />
      </div>
    </Layout>
  )
}
```

### Create Sign-Up Page

Create `src/pages/signup.tsx`:

```typescript
import Layout from "@theme/Layout"
import { SignUp } from "../components/Auth/SignUp"

export default function SignUpPage() {
  return (
    <Layout title="Sign Up">
      <div className="container margin-vert--lg">
        <SignUp />
      </div>
    </Layout>
  )
}
```

### Start Docusaurus

```bash
npm start
```

Docusaurus runs on `http://localhost:3001` (or configured port).

## Step 4: Test Authentication Flow

1. **Start Auth Service**: `cd Auth && npm run dev`
2. **Start Docusaurus**: `npm start` (in repo root)
3. **Navigate to Sign Up**: `http://localhost:3001/signup`
4. **Create Account**: Enter email and password
5. **Verify Sign In**: Should be automatically signed in
6. **Test Sign Out**: Click sign out button
7. **Test Sign In**: Sign in with created credentials

## Step 5: Test Personalization Features

### Progress Tracking

1. Sign in to your account
2. Navigate to any module section
3. Check browser console for progress tracking API calls
4. Visit dashboard to see progress

### Bookmarks

1. Navigate to a module section
2. Click bookmark button
3. Verify bookmark is saved
4. Visit dashboard to see bookmarks

### Notes

1. Navigate to a module section
2. Click "Add Note"
3. Enter note content and save
4. Refresh page to verify note persists

## Troubleshooting

### CORS Errors

- Verify `FRONTEND_URL` in Auth service matches Docusaurus URL
- Check CORS middleware is configured correctly
- Ensure credentials are enabled in CORS config

### Database Connection Errors

- Verify `DATABASE_URL` is correct
- Check database is accessible from your network
- Ensure database user has proper permissions

### Session Not Persisting

- Check cookies are being set (browser DevTools)
- Verify `BETTER_AUTH_SECRET` is set
- Ensure `baseURL` matches in both client and server

### Migration Errors

- Ensure database is empty or use `--force` flag
- Check database user has CREATE TABLE permissions
- Verify PostgreSQL version is compatible (12+)

## Next Steps

- Implement personalization components (see `tasks.md`)
- Set up Vercel deployment for auth service
- Configure production environment variables
- Set up email service for password reset
- Configure cloud storage for downloadable resources

## Development Workflow

1. **Make Changes**: Edit code in `Auth/src/` or `src/`
2. **Test Locally**: Run both services and test changes
3. **Commit**: Commit changes to feature branch
4. **Deploy**: Deploy auth service to Vercel (staging first)
5. **Test Production**: Test with production URLs
6. **Merge**: Merge to main branch after testing

## Environment Variables Reference

### Auth Service

- `DATABASE_URL` - PostgreSQL connection string
- `BETTER_AUTH_SECRET` - Random secret for Better Auth
- `BETTER_AUTH_URL` - Auth service URL
- `FRONTEND_URL` - Docusaurus frontend URL
- `EMAIL_SERVER` - SMTP server for emails

### Docusaurus Frontend

- `NEXT_PUBLIC_AUTH_URL` - Auth service URL (public, used in browser)

## Production Deployment

See deployment documentation for:
- Vercel configuration for auth service
- Environment variable setup
- Database connection in production
- CORS configuration for production domains

