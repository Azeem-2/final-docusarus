/**
 * TranslationButton Component
 * 
 * Adds a button above chapter titles to translate page content to Urdu.
 * Uses Cloudflare Worker API for translation (English → Urdu).
 * 
 * Worker URL: https://my-worker.translate-worker.workers.dev
 */

import React, { useState, useEffect, useRef } from 'react';
import useIsBrowser from '@docusaurus/useIsBrowser';
import { useAuth } from '@site/src/components/AuthProvider';
import AuthModal from '@site/src/components/AuthModal';
import { useToast } from '@site/src/components/Toast';

interface TranslationButtonProps {
  className?: string;
}

export default function TranslationButton({ className = '' }: TranslationButtonProps) {
  const { isAuthenticated, isLoading } = useAuth();
  const { showToast } = useToast();
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [originalContent, setOriginalContent] = useState<Map<HTMLElement, string>>(new Map());
  const [error, setError] = useState<string | null>(null);
  const [authModalOpen, setAuthModalOpen] = useState(false);
  const contentRef = useRef<HTMLElement | null>(null);
  const pageIdRef = useRef<string>('');
  const isBrowser = useIsBrowser();

  // Find the main content area
  useEffect(() => {
    const updateContentRef = () => {
      const article = document.querySelector('article') || document.querySelector('.markdown');
      contentRef.current = article;
      
      // Reset translation state when page content changes
      const currentPageId = window.location.pathname;
      if (pageIdRef.current !== currentPageId) {
        pageIdRef.current = currentPageId;
        setIsTranslated(false);
        setOriginalContent(new Map());
        setError(null);
      }
    };

    updateContentRef();

    // Watch for content changes (navigation)
    const contentObserver = new MutationObserver(updateContentRef);
    if (document.body) {
      contentObserver.observe(document.body, {
        childList: true,
        subtree: true,
      });
    }

    return () => {
      contentObserver.disconnect();
    };
  }, []);

  const translateToUrdu = async () => {
    // Check authentication first
    if (!isAuthenticated) {
      setAuthModalOpen(true);
      showToast('Please sign in to use the translation feature', 'info', 3000);
      return;
    }

    if (!contentRef.current) return;

    setIsTranslating(true);

    try {
      // If already translated, restore original
      if (isTranslated && originalContent.size > 0) {
        originalContent.forEach((text, element) => {
          element.textContent = text;
        });
        setIsTranslated(false);
        setIsTranslating(false);
        return;
      }

      // Store original content
      const elementsToTranslate = contentRef.current.querySelectorAll(
        'p, h1, h2, h3, h4, h5, h6, li, blockquote, td, th, span:not(.code-line):not([class*="token"])'
      );

      const contentMap = new Map<HTMLElement, string>();
      elementsToTranslate.forEach((el) => {
        const htmlEl = el as HTMLElement;
        // Skip code blocks, buttons, and navigation
        if (
          htmlEl.closest('pre') ||
          htmlEl.closest('code') ||
          htmlEl.closest('nav') ||
          htmlEl.closest('button') ||
          htmlEl.closest('.navbar') ||
          htmlEl.textContent?.trim().length === 0
        ) {
          return;
        }
        const text = htmlEl.textContent || '';
        if (text.trim().length > 0) {
          contentMap.set(htmlEl, text);
        }
      });

      setOriginalContent(contentMap);

      // Cloudflare Worker URL for translation
      const WORKER_URL = "https://my-worker.translate-worker.workers.dev";

      // Single text translation using Cloudflare Worker
      const translateText = async (text: string, retryCount = 0): Promise<string> => {
        const maxRetries = 2;
        
        try {
          const response = await fetch(WORKER_URL, {
            method: "POST",
            headers: {
              "Content-Type": "application/json",
            },
            body: JSON.stringify({ text }),
          });

          if (!response.ok) {
            // Provide helpful error message for CORS issues
            if (response.status === 500) {
              throw new Error(`Worker error (${response.status}): The worker may not be handling CORS preflight requests. Check worker logs and see CLOUDFLARE_WORKER_CORS_FIX.md`);
            }
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
          }

          const data = await response.json();
          
          if (data.translation && data.translation !== text && data.translation.length > 0) {
            return data.translation;
          }

          // If translation is same as original or empty, retry
          if (retryCount < maxRetries) {
            await new Promise(resolve => setTimeout(resolve, 1000 * (retryCount + 1)));
            return translateText(text, retryCount + 1);
          }

          return text; // Fallback to original if translation fails
        } catch (error) {
          console.warn('Translation error:', error);
          if (retryCount < maxRetries) {
            await new Promise(resolve => setTimeout(resolve, 1000 * (retryCount + 1)));
            return translateText(text, retryCount + 1);
          }
          return text; // Fallback to original
        }
      };

      // Batch translation for multiple texts (optimized for large content)
      const translateBatch = async (texts: string[]): Promise<string[]> => {
        try {
          const response = await fetch(WORKER_URL, {
            method: "POST",
            headers: {
              "Content-Type": "application/json",
            },
            body: JSON.stringify({ batch: texts }),
          });

          if (!response.ok) {
            // Provide helpful error message for CORS issues
            if (response.status === 500) {
              throw new Error(`Worker error (${response.status}): The worker may not be handling CORS preflight requests. Check worker logs and see CLOUDFLARE_WORKER_CORS_FIX.md`);
            }
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
          }

          const data = await response.json();
          
          if (data.translations && Array.isArray(data.translations)) {
            return data.translations;
          }

          throw new Error('Invalid batch translation response');
        } catch (error) {
          console.warn('Batch translation error:', error);
          // Fallback to individual translations
          return Promise.all(texts.map(text => translateText(text)));
        }
      };

      // Translate elements using batch API for better performance
      const elements = Array.from(contentMap.keys());
      const textsToTranslate: { element: HTMLElement; text: string; index: number }[] = [];
      
      // Collect all texts that need translation
      elements.forEach((element) => {
        const originalText = contentMap.get(element)!;
        // Only translate if text is meaningful (more than 3 chars and less than 500 chars)
        if (originalText.length > 3 && originalText.length < 500) {
          textsToTranslate.push({ element, text: originalText, index: textsToTranslate.length });
        }
      });

      const totalElements = textsToTranslate.length;
      console.log(`Starting translation of ${totalElements} elements using Cloudflare Worker...`);

      let translatedCount = 0;
      let failedCount = 0;

      // Use batch translation for better performance (process in chunks to avoid payload limits)
      const batchSize = 10; // Process 10 texts at a time
      
      for (let i = 0; i < textsToTranslate.length; i += batchSize) {
        const batch = textsToTranslate.slice(i, i + batchSize);
        const batchTexts = batch.map(item => item.text);
        
        try {
          // Use batch API for this chunk
          const translations = await translateBatch(batchTexts);
          
          // Apply translations to elements
          batch.forEach((item, idx) => {
            const translated = translations[idx];
            if (translated && translated !== item.text && translated.trim().length > 0) {
              item.element.textContent = translated;
              translatedCount++;
            } else {
              failedCount++;
              console.warn('Translation returned original text or empty:', item.text.substring(0, 50));
            }
          });

          // Log progress
          const progress = Math.min(i + batchSize, totalElements);
          console.log(`Translation progress: ${progress}/${totalElements} (${translatedCount} translated, ${failedCount} failed)`);
          
          // Small delay between batches to avoid overwhelming the worker
          if (i + batchSize < textsToTranslate.length) {
            await new Promise((resolve) => setTimeout(resolve, 200));
          }
        } catch (err) {
          console.warn('Batch translation failed, falling back to individual translations:', err);
          
          // Fallback to individual translations for this batch
          for (const item of batch) {
            try {
              const translated = await translateText(item.text);
              if (translated && translated !== item.text && translated.trim().length > 0) {
                item.element.textContent = translated;
                translatedCount++;
              } else {
                failedCount++;
              }
              await new Promise((resolve) => setTimeout(resolve, 100));
            } catch (individualErr) {
              failedCount++;
              console.warn('Failed to translate element:', individualErr);
            }
          }
        }
      }

      if (translatedCount > 0) {
        setIsTranslated(true);
        setError(null);
        console.log(`Translation complete: ${translatedCount} elements translated`);
      } else {
        setError('Translation failed. The API may be rate-limited. Please try again in a few moments.');
        setIsTranslated(false);
        console.error('Translation failed: No elements were translated');
      }
    } catch (error) {
      console.error('Translation failed:', error);
      setError('Translation failed. Please try again or use browser translation.');
      setIsTranslated(false);
    } finally {
      setIsTranslating(false);
    }
  };

  // Use CSS classes instead of inline styles based on colorMode to avoid hydration issues
  const buttonClassName = `translation-button ${className}`;

  // Don't render if still loading auth state
  if (isLoading) {
    return null;
  }

  return (
    <div style={{ position: 'relative', display: 'inline-block' }}>
      <button
        onClick={translateToUrdu}
        disabled={isTranslating || !isAuthenticated}
        className={buttonClassName}
        aria-label={isTranslated ? 'Show original text' : 'Translate to Urdu'}
        title={!isAuthenticated 
          ? 'Sign in to use translation feature' 
          : isTranslated 
            ? 'Click to show original text' 
            : 'Click to translate page to Urdu'}
      >
      {isTranslating ? (
        <>
          <span style={{ display: 'inline-block', animation: 'spin 1s linear infinite' }}>
            ⟳
          </span>
          <span>Translating...</span>
        </>
      ) : isTranslated ? (
        <>
          <span>🔙</span>
          <span>Show Original</span>
        </>
      ) : (
        <>
          <span>🌐</span>
          <span>Translate to Urdu</span>
        </>
      )}
      </button>
      {error && !isTranslating && (
        <div className="translation-error">
          {error}
        </div>
      )}
      <AuthModal
        isOpen={authModalOpen}
        onClose={() => setAuthModalOpen(false)}
        initialMode="signin"
        onSuccess={() => {
          showToast('Welcome! You can now use the translation feature', 'success', 3000);
        }}
        onError={(error) => {
          showToast(error, 'error', 5000);
        }}
      />
      <style>{`
        @keyframes spin {
          from { transform: rotate(0deg); }
          to { transform: rotate(360deg); }
        }
        .translation-button {
          display: inline-flex;
          align-items: center;
          gap: 8px;
          padding: 8px 16px;
          font-size: 14px;
          font-weight: 500;
          border-radius: 8px;
          border: 1px solid #e2e8f0;
          background-color: #ffffff;
          color: #2d3748;
          cursor: pointer;
          transition: all 0.2s ease;
          box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        [data-theme='dark'] .translation-button {
          border-color: #4a5568;
          background-color: #2d3748;
          color: #e2e8f0;
          box-shadow: 0 2px 4px rgba(0,0,0,0.3);
        }
        .translation-button:hover:not(:disabled) {
          background-color: #f7fafc;
          border-color: #cbd5e0;
          transform: translateY(-1px);
          box-shadow: 0 4px 6px rgba(0,0,0,0.15);
        }
        [data-theme='dark'] .translation-button:hover:not(:disabled) {
          background-color: #4a5568;
          border-color: #718096;
          box-shadow: 0 4px 6px rgba(0,0,0,0.4);
        }
        .translation-button:disabled {
          cursor: wait;
        }
        .translation-error {
          position: absolute;
          top: 100%;
          left: 0;
          margin-top: 8px;
          padding: 8px 12px;
          background-color: #fee2e2;
          color: #991b1b;
          border-radius: 6px;
          font-size: 12px;
          max-width: 300px;
          z-index: 10000;
          box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        [data-theme='dark'] .translation-error {
          background-color: #dc2626;
          color: #ffffff;
        }
      `}</style>
    </div>
  );
}

