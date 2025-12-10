/**
 * Root component for Docusaurus theme
 * Wraps all pages to inject global components like ChatWidget and AuthProvider
 */

import type {ReactNode} from 'react';
import {useEffect} from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ChatWidget from '@site/src/components/ChatWidget';
import {AuthProvider} from '@site/src/components/Auth/AuthProvider';

export default function Root({children}: {children: ReactNode}): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  
  // Set auth URL in window for easy access from anywhere (including navbar-inject)
  useEffect(() => {
    if (typeof window !== 'undefined') {
      const authUrl = (siteConfig.customFields?.authUrl as string) || 'http://localhost:3000';
      (window as any).__AUTH_URL__ = authUrl;
      console.log('[Root] Set __AUTH_URL__ to:', authUrl);
    }
  }, [siteConfig]);

  return (
    <AuthProvider>
      {children}
      <ChatWidget />
    </AuthProvider>
  );
}

