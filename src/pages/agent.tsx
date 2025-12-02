import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useIsBrowser from '@docusaurus/useIsBrowser';
import styles from './agent.module.css';

// ChatKit must be rendered client-side only
function AgentPageContent(): React.ReactNode {
  const [theme, setTheme] = useState<'dark' | 'light'>('dark');
  const isBrowser = useIsBrowser();
  
  // Get theme from data-theme attribute after hydration to avoid mismatch
  useEffect(() => {
    if (isBrowser) {
      const html = document.documentElement;
      const currentTheme = html.getAttribute('data-theme') === 'dark' ? 'dark' : 'light';
      setTheme(currentTheme);
      
      // Watch for theme changes
      const observer = new MutationObserver(() => {
        const newTheme = html.getAttribute('data-theme') === 'dark' ? 'dark' : 'light';
        setTheme(newTheme);
      });
      
      observer.observe(html, {
        attributes: true,
        attributeFilter: ['data-theme'],
      });
      
      return () => observer.disconnect();
    }
  }, [isBrowser]);
  
  // Dynamic import to avoid SSR issues
  const ChatKit = React.lazy(
    () => import('@site/src/components/ChatKit')
  );

  return (
    <div className={styles.agentPageContainer}>
      <React.Suspense fallback={<div className={styles.loading}>Loading agent...</div>}>
        <ChatKit 
          theme={theme} 
          height="calc(100vh - 80px)"
          greeting="Hello! I'm your AI agent. How can I help you today?"
          prompts={[
            { icon: '💡', label: 'Explain a concept', prompt: 'Explain how machine learning works in simple terms' },
            { icon: '💻', label: 'Write code', prompt: 'Write a Python function that sorts a list' },
            { icon: '📊', label: 'Analyze data', prompt: 'Help me understand best practices for data analysis' },
            { icon: '✍️', label: 'Help me write', prompt: 'Help me write a professional email' },
          ]}
        />
      </React.Suspense>
    </div>
  );
}

export default function AgentPage(): React.ReactNode {
  return (
    <Layout
      title="AI Agent"
      description="Chat with our AI agent powered by Gemini">
      <BrowserOnly fallback={<div className={styles.loading}>Loading...</div>}>
        {() => <AgentPageContent />}
      </BrowserOnly>
    </Layout>
  );
}

