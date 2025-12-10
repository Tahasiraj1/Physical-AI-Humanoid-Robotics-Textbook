import React from 'react';
import Layout from '@theme/Layout';
import { AuthProvider } from '@site/src/components/Auth/AuthProvider';
import SignUp from '@site/src/components/Auth/SignUp';

export default function SignUpPage(): JSX.Element {
  return (
    <AuthProvider>
      <Layout
        title="Sign Up"
        description="Create an account to access personalized features">
        <div className="container margin-vert--lg">
          <SignUp />
        </div>
      </Layout>
    </AuthProvider>
  );
}

