import React from 'react';
import Layout from '@theme/Layout';
import { AuthProvider } from '@site/src/components/Auth/AuthProvider';
import SignIn from '@site/src/components/Auth/SignIn';

export default function SignInPage(): JSX.Element {
  return (
    <AuthProvider>
      <Layout
        title="Sign In"
        description="Sign in to your account">
        <div className="container margin-vert--lg">
          <SignIn />
        </div>
      </Layout>
    </AuthProvider>
  );
}

