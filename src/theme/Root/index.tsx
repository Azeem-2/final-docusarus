import React from 'react';
import { PyodideProvider } from '@/contexts/PyodideContext';
import { AnalyticsTracker } from '@/components/AnalyticsTracker';
import { ChatWidget } from '@/components/ChatWidget';
import { AuthProvider } from '@site/src/components/AuthProvider';
import { ToastProvider } from '@site/src/components/Toast';

export default function Root({ children }: { children: React.ReactNode }) {
  // Prevent FOUC by setting theme immediately before React hydrates
  if (typeof window !== 'undefined') {
    const html = document.documentElement;
    // Force dark mode and prevent any theme switching
    html.setAttribute('data-theme', 'dark');
    html.setAttribute('data-theme-choice', 'dark');
    // Remove any existing theme classes that might conflict
    html.classList.remove('light-theme', 'dark-theme');
    html.classList.add('dark-theme');
    
    // Override localStorage to prevent theme changes
    try {
      localStorage.setItem('theme', 'dark');
      localStorage.setItem('docusaurus-theme', 'dark');
    } catch (e) {
      // Ignore localStorage errors
    }
  }

  return (
    <PyodideProvider>
      <AnalyticsTracker>
        <AuthProvider>
          <ToastProvider>
            {children}
            <ChatWidget />
          </ToastProvider>
        </AuthProvider>
      </AnalyticsTracker>
    </PyodideProvider>
  );
}
