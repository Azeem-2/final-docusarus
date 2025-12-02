import React, { useState, useRef, useEffect, useCallback, type ReactNode } from 'react';
import styles from './styles.module.css';

// Types
interface Message {
  id: string;
  role: 'user' | 'assistant' | 'system';
  content: string;
  timestamp: Date;
  status?: 'sending' | 'sent' | 'error';
  widgets?: Widget[];
}

interface Widget {
  type: 'card' | 'code' | 'image' | 'button' | 'list';
  data: Record<string, unknown>;
}

interface Prompt {
  label: string;
  prompt: string;
  icon?: string;
}

interface ChatKitProps {
  /** API endpoint URL */
  apiUrl?: string;
  /** Theme mode */
  theme?: 'light' | 'dark' | 'system';
  /** Container height */
  height?: string;
  /** Welcome greeting */
  greeting?: string;
  /** Suggested prompts */
  prompts?: Prompt[];
  /** Placeholder text */
  placeholder?: string;
  /** Enable markdown rendering */
  markdown?: boolean;
  /** Show timestamps */
  showTimestamps?: boolean;
  /** Show typing indicator */
  showTypingIndicator?: boolean;
  /** Enable conversation history */
  enableHistory?: boolean;
  /** Max history messages to send */
  maxHistoryMessages?: number;
  /** Callback on message sent */
  onMessageSent?: (message: Message) => void;
  /** Callback on response received */
  onResponseReceived?: (message: Message) => void;
  /** Callback on error */
  onError?: (error: Error) => void;
}

// Markdown-like text renderer (simple version)
function renderMarkdown(text: string): ReactNode {
  // Split by code blocks first
  const parts = text.split(/(```[\s\S]*?```)/g);
  
  return parts.map((part, index) => {
    // Code block
    if (part.startsWith('```') && part.endsWith('```')) {
      const lines = part.slice(3, -3).split('\n');
      const language = lines[0].trim();
      const code = lines.slice(language ? 1 : 0).join('\n');
      return (
        <div key={index} className={styles.codeBlock}>
          {language && <div className={styles.codeLanguage}>{language}</div>}
          <pre><code>{code}</code></pre>
          <button 
            className={styles.copyButton}
            onClick={() => navigator.clipboard.writeText(code)}
            title="Copy code"
          >
            📋
          </button>
        </div>
      );
    }
    
    // Inline formatting
    let formatted = part
      // Bold
      .replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>')
      // Italic
      .replace(/\*(.*?)\*/g, '<em>$1</em>')
      // Inline code
      .replace(/`([^`]+)`/g, '<code class="inline-code">$1</code>')
      // Links
      .replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2" target="_blank" rel="noopener">$1</a>')
      // Line breaks
      .replace(/\n/g, '<br/>');
    
    return <span key={index} dangerouslySetInnerHTML={{ __html: formatted }} />;
  });
}

// Format timestamp
function formatTime(date: Date): string {
  return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
}

// Generate unique ID
function generateId(): string {
  return `msg_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`;
}

// Default prompts
const defaultPrompts: Prompt[] = [
  { icon: '💡', label: 'Explain a concept', prompt: 'Explain how machine learning works in simple terms' },
  { icon: '💻', label: 'Write code', prompt: 'Write a Python function that reverses a string' },
  { icon: '📊', label: 'Analyze data', prompt: 'Help me understand the best practices for data analysis' },
  { icon: '✍️', label: 'Help me write', prompt: 'Help me write a professional email' },
];

export default function ChatKit({
  apiUrl = 'http://localhost:8000',
  theme = 'light',
  height = '100%',
  greeting = 'Hello! How can I help you today?',
  prompts = defaultPrompts,
  placeholder = 'Message AI...',
  markdown = true,
  showTimestamps = true,
  showTypingIndicator = true,
  enableHistory = true,
  maxHistoryMessages = 20,
  onMessageSent,
  onResponseReceived,
  onError,
}: ChatKitProps): ReactNode {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isConnected, setIsConnected] = useState(true);
  
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom
  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  useEffect(() => {
    scrollToBottom();
  }, [messages, scrollToBottom]);

  // Auto-resize textarea
  useEffect(() => {
    if (inputRef.current) {
      inputRef.current.style.height = 'auto';
      inputRef.current.style.height = `${Math.min(inputRef.current.scrollHeight, 150)}px`;
    }
  }, [input]);

  // Health check
  useEffect(() => {
    const checkHealth = async () => {
      try {
        const response = await fetch(`${apiUrl}/health`);
        setIsConnected(response.ok);
      } catch {
        setIsConnected(false);
      }
    };
    checkHealth();
    const interval = setInterval(checkHealth, 30000);
    return () => clearInterval(interval);
  }, [apiUrl]);

  // Send message
  const sendMessage = async (content?: string) => {
    const messageContent = content || input.trim();
    if (!messageContent || isLoading) return;

    const userMessage: Message = {
      id: generateId(),
      role: 'user',
      content: messageContent,
      timestamp: new Date(),
      status: 'sending',
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);
    setError(null);

    // Update status to sent
    setMessages(prev => 
      prev.map(m => m.id === userMessage.id ? { ...m, status: 'sent' } : m)
    );

    onMessageSent?.(userMessage);

    try {
      // Build conversation history
      const history = enableHistory 
        ? messages.slice(-maxHistoryMessages).map(msg => ({
            role: msg.role,
            content: msg.content,
          }))
        : undefined;

      const response = await fetch(`${apiUrl}/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          message: messageContent,
          conversation_history: history && history.length > 0 ? history : undefined,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.error || `Request failed: ${response.status}`);
      }

      const data = await response.json();
      const assistantContent = data.response || data.content || 'No response received';

      const assistantMessage: Message = {
        id: generateId(),
        role: 'assistant',
        content: assistantContent,
        timestamp: new Date(),
        status: 'sent',
      };

      setMessages(prev => [...prev, assistantMessage]);
      onResponseReceived?.(assistantMessage);
      
    } catch (err) {
      const error = err instanceof Error ? err : new Error('Unknown error');
      console.error('Chat error:', error);
      setError(error.message);
      onError?.(error);
      
      // Mark user message as error
      setMessages(prev => 
        prev.map(m => m.id === userMessage.id ? { ...m, status: 'error' } : m)
      );
    } finally {
      setIsLoading(false);
      inputRef.current?.focus();
    }
  };

  // Handle keyboard
  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  // Clear conversation
  const clearConversation = () => {
    setMessages([]);
    setError(null);
  };

  // Retry failed message
  const retryMessage = (messageId: string) => {
    const message = messages.find(m => m.id === messageId);
    if (message && message.status === 'error') {
      setMessages(prev => prev.filter(m => m.id !== messageId));
      sendMessage(message.content);
    }
  };

  // Get theme class
  const themeClass = theme === 'system' 
    ? (window.matchMedia('(prefers-color-scheme: dark)').matches ? styles.dark : '')
    : (theme === 'dark' ? styles.dark : '');

  return (
    <div 
      ref={containerRef}
      className={`${styles.chatkit} ${themeClass}`} 
      style={{ height }}
    >
      {/* Header */}
      <div className={styles.header}>
        <div className={styles.headerLeft}>
          <div className={styles.logo}>
            <span className={styles.logoIcon}>✨</span>
            <span className={styles.logoText}>AI Chat</span>
          </div>
          <div className={`${styles.status} ${isConnected ? styles.connected : styles.disconnected}`}>
            <span className={styles.statusDot}></span>
            <span>{isConnected ? 'Connected' : 'Disconnected'}</span>
          </div>
        </div>
        <div className={styles.headerRight}>
          {messages.length > 0 && (
            <button 
              className={styles.headerButton}
              onClick={clearConversation}
              title="New conversation"
            >
              🔄 New Chat
            </button>
          )}
        </div>
      </div>

      {/* Messages */}
      <div className={styles.messages}>
        {messages.length === 0 ? (
          <div className={styles.welcome}>
            <div className={styles.welcomeIcon}>🤖</div>
            <h2 className={styles.welcomeTitle}>{greeting}</h2>
            <p className={styles.welcomeSubtitle}>Powered by Gemini AI</p>
            
            <div className={styles.promptGrid}>
              {prompts.map((item, index) => (
                <button
                  key={index}
                  className={styles.promptCard}
                  onClick={() => sendMessage(item.prompt)}
                >
                  <span className={styles.promptIcon}>{item.icon || '💬'}</span>
                  <span className={styles.promptLabel}>{item.label}</span>
                </button>
              ))}
            </div>
          </div>
        ) : (
          <div className={styles.messageList}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.messageRow} ${styles[message.role]}`}
              >
                <div className={styles.avatar}>
                  {message.role === 'user' ? '👤' : '🤖'}
                </div>
                <div className={styles.messageWrapper}>
                  <div className={styles.messageHeader}>
                    <span className={styles.messageSender}>
                      {message.role === 'user' ? 'You' : 'AI Assistant'}
                    </span>
                    {showTimestamps && (
                      <span className={styles.messageTime}>
                        {formatTime(message.timestamp)}
                      </span>
                    )}
                  </div>
                  <div className={styles.messageContent}>
                    {markdown ? renderMarkdown(message.content) : message.content}
                  </div>
                  {message.status === 'error' && (
                    <div className={styles.messageError}>
                      <span>Failed to send</span>
                      <button onClick={() => retryMessage(message.id)}>Retry</button>
                    </div>
                  )}
                  <div className={styles.messageActions}>
                    <button 
                      onClick={() => navigator.clipboard.writeText(message.content)}
                      title="Copy message"
                    >
                      📋
                    </button>
                  </div>
                </div>
              </div>
            ))}
            
            {/* Typing indicator */}
            {isLoading && showTypingIndicator && (
              <div className={`${styles.messageRow} ${styles.assistant}`}>
                <div className={styles.avatar}>🤖</div>
                <div className={styles.messageWrapper}>
                  <div className={styles.typing}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            
            <div ref={messagesEndRef} />
          </div>
        )}
      </div>

      {/* Error banner */}
      {error && (
        <div className={styles.errorBanner}>
          <span>⚠️ {error}</span>
          <button onClick={() => setError(null)}>✕</button>
        </div>
      )}

      {/* Composer */}
      <div className={styles.composer}>
        <div className={styles.composerInner}>
          <textarea
            ref={inputRef}
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder={placeholder}
            className={styles.textarea}
            rows={1}
            disabled={isLoading}
          />
          <div className={styles.composerActions}>
            <button
              onClick={() => sendMessage()}
              disabled={!input.trim() || isLoading}
              className={styles.sendButton}
              title="Send message"
            >
              {isLoading ? (
                <span className={styles.spinner}></span>
              ) : (
                <svg viewBox="0 0 24 24" fill="currentColor" width="20" height="20">
                  <path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z"/>
                </svg>
              )}
            </button>
          </div>
        </div>
        <div className={styles.composerHint}>
          <span>Enter to send • Shift+Enter for new line</span>
          {enableHistory && messages.length > 0 && (
            <span className={styles.historyIndicator}>
              💬 {messages.length} messages in context
            </span>
          )}
        </div>
      </div>
    </div>
  );
}

