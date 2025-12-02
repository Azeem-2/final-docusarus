import React, {type ReactNode, useEffect} from 'react';
import Layout from '@theme-original/DocItem/Layout';
import type LayoutType from '@theme/DocItem/Layout';
import type {WrapperProps} from '@docusaurus/types';
import {createRoot} from 'react-dom/client';
import TranslationButton from '@site/src/components/TranslationButton';
import { AuthProvider } from '@site/src/components/AuthProvider';
import { ToastProvider } from '@site/src/components/Toast';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): ReactNode {
  useEffect(() => {
    // Find the h1 title element and insert translation button above it
    const findAndInsertButton = () => {
      // Look for the main article h1 (chapter title)
      const article = document.querySelector('article');
      if (!article) return;

      const h1 = article.querySelector('h1');
      if (!h1) return;

      // Check if button already exists
      if (h1.previousElementSibling?.classList.contains('translation-button-container')) {
        return;
      }

      // Create container for translation button
      const buttonContainer = document.createElement('div');
      buttonContainer.className = 'translation-button-container';
      buttonContainer.style.cssText = `
        margin-bottom: 1.5rem;
        display: flex;
        justify-content: flex-start;
        align-items: center;
      `;

      // Render the button with providers wrapped
      // Since this is rendered inside the Root component which already has providers,
      // we need to check if we're within the provider context
      // Actually, since Root wraps everything, the providers should be available
      // But createRoot creates a new root, so we need to wrap it
      const root = createRoot(buttonContainer);
      root.render(
        <AuthProvider>
          <ToastProvider>
            <TranslationButton />
          </ToastProvider>
        </AuthProvider>
      );

      // Insert before h1
      h1.parentNode?.insertBefore(buttonContainer, h1);
    };

    // Wait for content to render
    const timer = setTimeout(findAndInsertButton, 300);
    return () => clearTimeout(timer);
  }, []);

  return (
    <>
      <Layout {...props} />
    </>
  );
}
