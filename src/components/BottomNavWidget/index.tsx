import React, { useState } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import { useAuth } from '@site/src/components/AuthProvider';
import { useToast } from '@site/src/components/Toast';
import AuthModal from '@site/src/components/AuthModal';
import styles from './BottomNavWidget.module.css';

// Simplified TranslationButton for nav
function TranslationButtonNav() {
  const { isAuthenticated, isLoading } = useAuth();
  const { showToast } = useToast();
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [authModalOpen, setAuthModalOpen] = useState(false);

  const handleTranslate = async () => {
    if (!isAuthenticated) {
      setAuthModalOpen(true);
      showToast('Please sign in to use the translation feature', 'info', 3000);
      return;
    }

    // Trigger the main translation button if it exists
    const mainTranslationButton = document.querySelector('.translation-button');
    if (mainTranslationButton) {
      (mainTranslationButton as HTMLElement).click();
    } else {
      showToast('Translation feature is loading...', 'info', 2000);
    }
  };

  if (isLoading) {
    return (
      <button className={styles.navItem} aria-label="Translation" disabled>
        <svg className={styles.navIcon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <circle cx="12" cy="12" r="10" />
          <line x1="12" y1="2" x2="12" y2="6" />
          <line x1="12" y1="18" x2="12" y2="22" />
        </svg>
      </button>
    );
  }

  return (
    <>
      <button 
        className={styles.navItem} 
        onClick={handleTranslate}
        aria-label="Translate"
        title={isAuthenticated ? 'Translate to Urdu' : 'Sign in to translate'}
      >
        <svg className={styles.navIcon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <circle cx="12" cy="12" r="10" />
          <line x1="2" y1="12" x2="22" y2="12" />
          <path d="M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z" />
        </svg>
      </button>
      <AuthModal
        isOpen={authModalOpen}
        onClose={() => setAuthModalOpen(false)}
        initialMode="signin"
        onSuccess={() => {
          showToast('Welcome! You can now use the translation feature', 'success', 3000);
        }}
      />
    </>
  );
}

export default function BottomNavWidget() {
  const history = useHistory();
  const location = useLocation();
  const { isAuthenticated } = useAuth();

  const handleHome = () => {
    history.push('/');
  };

  const handleProfile = () => {
    if (isAuthenticated) {
      // Navigate to profile or show profile menu
      const profileButton = document.querySelector('[data-profile-button]');
      if (profileButton) {
        (profileButton as HTMLElement).click();
      }
    } else {
      // Show auth modal
      const loginButton = document.querySelector('[data-login-button]');
      if (loginButton) {
        (loginButton as HTMLElement).click();
      }
    }
  };

  const handleMessage = () => {
    // Toggle chat widget - find the toggle button
    // The ChatWidget uses className={styles.toggleButton} which becomes a hash
    // Try multiple selectors to find it
    const selectors = [
      '[aria-label*="chat" i]',
      '[aria-label*="Sign in to access chat" i]',
      '[aria-label*="Open chat" i]',
      '[aria-label*="Close chat" i]',
      'button[class*="toggle"]',
    ];
    
    let chatButton: HTMLElement | null = null;
    for (const selector of selectors) {
      chatButton = document.querySelector(selector) as HTMLElement;
      if (chatButton) break;
    }
    
    if (chatButton) {
      chatButton.click();
    } else {
      // Fallback: try to find any button in chat widget area
      const chatWidgets = document.querySelectorAll('[class*="widget"], [class*="chat"]');
      for (const widget of Array.from(chatWidgets)) {
        const button = widget.querySelector('button');
        if (button) {
          button.click();
          break;
        }
      }
    }
  };

  const handleSettings = () => {
    // Could navigate to settings page or show settings modal
    console.log('Settings clicked');
  };

  return (
    <nav className={styles.bottomNav}>
      <button 
        className={styles.navItem} 
        onClick={handleHome}
        aria-label="Home"
      >
        <svg className={styles.navIcon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <path d="M3 9l9-7 9 7v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z" />
          <polyline points="9 22 9 12 15 12 15 22" />
        </svg>
      </button>

      <button 
        className={styles.navItem} 
        onClick={handleProfile}
        aria-label="Profile"
      >
        <svg className={styles.navIcon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
          <circle cx="12" cy="7" r="4" />
        </svg>
      </button>

      <div className={styles.messageButtonWrapper}>
        <button 
          className={styles.messageButton} 
          onClick={handleMessage}
          aria-label="Message"
        >
          <svg className={styles.messageIcon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        </button>
        <span className={styles.messageLabel}>Message</span>
      </div>

      <div className={styles.translationButtonWrapper}>
        <TranslationButtonNav />
      </div>

      <button 
        className={styles.navItem} 
        onClick={handleSettings}
        aria-label="Settings"
      >
        <svg className={styles.navIcon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <circle cx="12" cy="12" r="3" />
          <path d="M12 1v6m0 6v6M5.64 5.64l4.24 4.24m4.24 4.24l4.24 4.24M1 12h6m6 0h6M5.64 18.36l4.24-4.24m4.24-4.24l4.24-4.24" />
        </svg>
      </button>
    </nav>
  );
}

