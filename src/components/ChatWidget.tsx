import React, { useState, useEffect, useRef, useCallback } from 'react';
import { useAuth } from '@site/src/components/AuthProvider';
import AuthModal from '@site/src/components/AuthModal';
import { useToast } from '@site/src/components/Toast';
import styles from './ChatWidget.module.css';

export const ChatWidget = () => {
    const { isAuthenticated, isLoading } = useAuth();
    const { showToast } = useToast();
    const [isOpen, setIsOpen] = useState(false);
    const [selectedText, setSelectedText] = useState<string>('');
    const [authModalOpen, setAuthModalOpen] = useState(false);
    const iframeRef = useRef<HTMLIFrameElement>(null);

    /**
     * Function to send selected text to iframe
     * 
     * The iframe at https://frontend-delta-pied-64.vercel.app needs to implement the following:
     * 
     * 1. Listen for postMessage events:
     *    window.addEventListener('message', (event) => {
     *      // Verify origin for security
     *      const allowedOrigins = ['http://localhost:3000', window.location.origin];
     *      if (!allowedOrigins.includes(event.origin)) return;
     * 
     *      if (event.data.type === 'SET_SELECTED_TEXT') {
     *        // Display the selected text in a visible area of the chatbot UI
     *        // Example: Show it in a banner/box above the input field
     *        // Store it in state so it can be included with user messages
     *        setSelectedText(event.data.text);
     *        displaySelectedText(event.data.text); // Show in UI
     *      }
     * 
     *      if (event.data.type === 'CLEAR_SELECTED_TEXT') {
     *        // Clear the displayed selected text
     *        setSelectedText('');
     *        hideSelectedTextDisplay();
     *      }
     *    });
     * 
     * 2. When user sends a message, combine selected text with user input:
     *    const sendMessage = () => {
     *      const fullMessage = selectedText 
     *        ? `${selectedText}\n\nUser question: ${userInput}`
     *        : userInput;
     *      // Send fullMessage to chat API
     *    };
     */
    const sendSelectedTextToIframe = useCallback((text: string) => {
        if (iframeRef.current?.contentWindow) {
            try {
                iframeRef.current.contentWindow.postMessage(
                    {
                        type: 'SET_SELECTED_TEXT',
                        text: text,
                        timestamp: Date.now()
                    },
                    'https://frontend-delta-pied-64.vercel.app'
                );
            } catch (error) {
                console.warn('Failed to send selected text to iframe:', error);
            }
        }
    }, []);

    // Listen for text selection on the page
    useEffect(() => {
        // Don't handle selection if not authenticated
        if (!isAuthenticated) {
            return;
        }

        const handleSelection = () => {
            const selection = window.getSelection();
            const text = selection?.toString().trim() || '';
            
            // Only process if there's meaningful text (more than 3 characters)
            // and it's not just whitespace
            if (text.length > 3 && text !== selectedText) {
                setSelectedText(text);
                
                // If chat is not open, open it when text is selected
                if (!isOpen) {
                    setIsOpen(true);
                } else {
                    // If chat is already open, send the selected text immediately
                    sendSelectedTextToIframe(text);
                }
            }
        };

        // Listen for mouseup (when selection ends) - this is more reliable
        const handleMouseUp = () => {
            // Small delay to ensure selection is complete
            setTimeout(handleSelection, 10);
        };

        document.addEventListener('mouseup', handleMouseUp);

        return () => {
            document.removeEventListener('mouseup', handleMouseUp);
        };
    }, [isOpen, selectedText, sendSelectedTextToIframe, isAuthenticated]);

    // Send selected text to iframe when it opens
    useEffect(() => {
        if (isOpen && selectedText && iframeRef.current) {
            // Wait a bit for iframe to load, then send message
            const timer = setTimeout(() => {
                sendSelectedTextToIframe(selectedText);
            }, 500);

            return () => clearTimeout(timer);
        }
    }, [isOpen, selectedText, sendSelectedTextToIframe]);

    const handleToggle = () => {
        // Check authentication before opening
        if (!isAuthenticated) {
            setAuthModalOpen(true);
            showToast('Please sign in to access the chat assistant', 'info', 3000);
            return;
        }

        setIsOpen(!isOpen);
        // Clear selection when closing
        if (isOpen) {
            setSelectedText('');
            // Notify iframe that selected text is cleared
            if (iframeRef.current?.contentWindow) {
                try {
                    iframeRef.current.contentWindow.postMessage(
                        {
                            type: 'CLEAR_SELECTED_TEXT'
                        },
                        'https://frontend-delta-pied-64.vercel.app'
                    );
                } catch (error) {
                    console.warn('Failed to clear selected text in iframe:', error);
                }
            }
        }
    };

    // Don't render if still loading auth state
    if (isLoading) {
        return null;
    }

    // Don't render chat widget if not authenticated
    if (!isAuthenticated) {
        return (
            <>
                <div className={styles.widgetContainer}>
                    <button
                        onClick={handleToggle}
                        className={styles.toggleButton}
                        aria-label="Sign in to access chat"
                        title="Sign in to access chat assistant"
                    >
                        <div className={styles.iconWrapper}>
                            <svg 
                                className={styles.chatIcon}
                                viewBox="0 0 24 24" 
                                fill="none" 
                                stroke="currentColor" 
                                strokeWidth="2.5" 
                                strokeLinecap="round" 
                                strokeLinejoin="round"
                            >
                                <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
                            </svg>
                        </div>
                    </button>
                </div>
                <AuthModal
                    isOpen={authModalOpen}
                    onClose={() => setAuthModalOpen(false)}
                    initialMode="signin"
                    onSuccess={() => {
                        showToast('Welcome! You can now access the chat assistant', 'success', 3000);
                    }}
                    onError={(error) => {
                        showToast(error, 'error', 5000);
                    }}
                />
            </>
        );
    }

    return (
        <div className={styles.widgetContainer}>
            {isOpen && (
                <div className={styles.chatFrame}>
                    <iframe
                        ref={iframeRef}
                        src="https://frontend-delta-pied-64.vercel.app/embed"
                        width="400"
                        height="600"
                        frameBorder="0"
                        title="Chat Assistant"
                    ></iframe>
                </div>
            )}
            <button
                onClick={handleToggle}
                className={`${styles.toggleButton} ${isOpen ? styles.open : ''}`}
                aria-label={isOpen ? "Close chat" : "Open chat"}
            >
                <div className={styles.iconWrapper}>
                    {isOpen ? (
                        // Beautiful Close Icon (X)
                        <div className={styles.closeIcon}></div>
                    ) : (
                        // Chat Icon
                        <svg 
                            className={styles.chatIcon}
                            viewBox="0 0 24 24" 
                            fill="none" 
                            stroke="currentColor" 
                            strokeWidth="2.5" 
                            strokeLinecap="round" 
                            strokeLinejoin="round"
                        >
                            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
                        </svg>
                    )}
                </div>
            </button>
        </div>
    );
};
