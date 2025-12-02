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
     * The iframe at https://chatkit-frontend-dusky.vercel.app needs to implement the following:
     * 
     * 1. Listen for postMessage events:
     *    window.addEventListener('message', (event) => {
     *      // Verify origin for security
     *      const allowedOrigins = ['http://localhost:3000', window.location.origin];
     *      if (!allowedOrigins.includes(event.origin)) return;
     * 
     *      if (event.data.type === 'SET_SELECTED_TEXT') {
     *        // IMPORTANT: Display the selected text at the TOP of the chatbot UI
     *        // Show it in a prominent banner/box above the chat messages area
     *        // Store it in state so it can be included with user messages
     *        setSelectedText(event.data.text);
     *        
     *        // Display at top of chatbot UI (above messages, below header)
     *        displaySelectedTextBanner(event.data.text);
     *        // Example UI structure:
     *        // <div className="selected-text-banner">
     *        //   <strong>📄 Selected Text:</strong>
     *        //   <p>{event.data.text}</p>
     *        //   <button onClick={clearSelectedText}>×</button>
     *        // </div>
     *      }
     * 
     *      if (event.data.type === 'CLEAR_SELECTED_TEXT') {
     *        // Clear the displayed selected text
     *        setSelectedText('');
     *        hideSelectedTextBanner();
     *      }
     *    });
     * 
     * 2. When user sends a message, AUTOMATICALLY append selected text:
     *    const sendMessage = () => {
     *      // ALWAYS combine selected text with user message
     *      const fullMessage = selectedText 
     *        ? `${selectedText}\n\n---\n\nUser question: ${userInput}`
     *        : userInput;
     *      
     *      // Send fullMessage to chat API (includes both selected text and user input)
     *      sendToAPI(fullMessage);
     *    };
     */
    const sendSelectedTextToIframe = useCallback((text: string) => {
        if (!text || text.trim().length === 0) return;
        
        const sendMessage = (attempt: number = 1) => {
            if (iframeRef.current?.contentWindow) {
                try {
                    const message = {
                        type: 'SET_SELECTED_TEXT',
                        text: text,
                        timestamp: Date.now(),
                        instruction: 'Display this text at the top of the chatbot UI and append it to user messages when sending',
                        displayAtTop: true,
                        appendToMessages: true
                    };
                    
                    // Send to specific origin
                    iframeRef.current.contentWindow.postMessage(
                        message,
                        'https://chatkit-frontend-dusky.vercel.app'
                    );
                    
                    // Also try with wildcard origin as fallback (less secure but more reliable)
                    try {
                        iframeRef.current.contentWindow.postMessage(
                            message,
                            '*'
                        );
                    } catch (e) {
                        // Ignore wildcard errors
                    }
                    
                    console.log(`[ChatWidget] Selected text sent to chatbot (attempt ${attempt}):`, {
                        text: text.substring(0, 50) + '...',
                        fullLength: text.length,
                        timestamp: new Date().toISOString()
                    });
                } catch (error) {
                    console.warn('[ChatWidget] Failed to send selected text to iframe:', error);
                }
            } else {
                console.warn('[ChatWidget] Iframe contentWindow not available, retrying...');
            }
        };

        // Try to send immediately
        sendMessage(1);

        // Multiple retries with increasing delays
        const retries = [100, 300, 500, 1000, 2000];
        const timers = retries.map((delay, index) => 
            setTimeout(() => sendMessage(index + 2), delay)
        );

        return () => {
            timers.forEach(timer => clearTimeout(timer));
        };
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
                }
                
                // Always send selected text immediately (will retry if chat is opening)
                if (isOpen && iframeRef.current) {
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

    // Send selected text to iframe when it opens or when iframe loads
    useEffect(() => {
        if (isOpen && selectedText && selectedText.length > 3) {
            // Wait for iframe to load, then send message with multiple retries
            const sendWithRetries = () => {
                console.log('[ChatWidget] Sending selected text with retries:', selectedText.substring(0, 50));
                // Multiple attempts to ensure message is received
                sendSelectedTextToIframe(selectedText);
            };

            // Check if iframe is already loaded
            if (iframeRef.current?.contentWindow) {
                // Small delay to ensure iframe is fully ready
                const timer = setTimeout(() => {
                    sendWithRetries();
                }, 100);
                return () => clearTimeout(timer);
            } else {
                // Wait for iframe to load
                const handleIframeLoad = () => {
                    console.log('[ChatWidget] Iframe loaded, sending selected text');
                    setTimeout(() => {
                        sendWithRetries();
                    }, 200);
                };
                
                if (iframeRef.current) {
                    iframeRef.current.addEventListener('load', handleIframeLoad);
                    return () => {
                        iframeRef.current?.removeEventListener('load', handleIframeLoad);
                    };
                }
            }
        }
    }, [isOpen, selectedText, sendSelectedTextToIframe]);

    // Also send selected text immediately when it changes (if chat is already open)
    useEffect(() => {
        if (selectedText && selectedText.length > 3 && isOpen && iframeRef.current?.contentWindow) {
            // Send immediately when text is selected and chat is open
            sendSelectedTextToIframe(selectedText);
        }
    }, [selectedText, isOpen, sendSelectedTextToIframe]);

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
                        'https://chatkit-frontend-dusky.vercel.app'
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
                        src="https://chatkit-frontend-dusky.vercel.app/embed"
                        width="400"
                        height="600"
                        frameBorder="0"
                        title="Chat Assistant"
                        onLoad={() => {
                            console.log('[ChatWidget] Iframe onLoad event fired');
                            // When iframe loads, send any selected text immediately
                            if (selectedText && selectedText.length > 3) {
                                console.log('[ChatWidget] Sending selected text on iframe load:', selectedText.substring(0, 50));
                                // Multiple attempts with delays
                                setTimeout(() => sendSelectedTextToIframe(selectedText), 200);
                                setTimeout(() => sendSelectedTextToIframe(selectedText), 500);
                                setTimeout(() => sendSelectedTextToIframe(selectedText), 1000);
                            }
                        }}
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
