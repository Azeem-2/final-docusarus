# Embedded Chatbot Integration Guide

## Overview
The parent page (`ChatWidget.tsx`) sends selected text to the embedded chatbot via `postMessage`. The embedded chatbot needs to:
1. Listen for these messages
2. Display the selected text in the UI
3. Include the selected text with user messages when sending to the API

## ✅ Implementation Status
**COMPLETED** - The feature has been implemented using the following approach:
- **Frontend**: SelectedTextDisplay component, Embed component with postMessage listener, ChatKitPanel with header injection
- **Backend**: Extracts `X-Selected-Text` header and prepends to user messages
- **Method**: Uses HTTP headers to pass selected text (clean separation of concerns)

## Implementation Steps

### Step 1: Add State for Selected Text

In your embedded chatbot component (the one running at `localhost:5171/embed`), add state to store the selected text:

```typescript
const [selectedText, setSelectedText] = useState<string>('');
```

### Step 2: Listen for postMessage Events

Add a `useEffect` hook to listen for messages from the parent window:

```typescript
useEffect(() => {
  const handleMessage = (event: MessageEvent) => {
    // Security: Verify the origin
    const allowedOrigins = [
      'http://localhost:3000',
      'http://localhost:5173', // Common dev server ports
      window.location.origin,
      // Add your production domain here
    ];

    if (!allowedOrigins.includes(event.origin)) {
      console.warn('Blocked message from unauthorized origin:', event.origin);
      return;
    }

    // Handle SET_SELECTED_TEXT message
    if (event.data.type === 'SET_SELECTED_TEXT') {
      const text = event.data.text;
      setSelectedText(text);
      console.log('Selected text received:', text);
    }

    // Handle CLEAR_SELECTED_TEXT message
    if (event.data.type === 'CLEAR_SELECTED_TEXT') {
      setSelectedText('');
      console.log('Selected text cleared');
    }
  };

  window.addEventListener('message', handleMessage);

  return () => {
    window.removeEventListener('message', handleMessage);
  };
}, []);
```

### Step 3: Display Selected Text in UI

Add a UI component to display the selected text above the input field. Example:

```tsx
{selectedText && (
  <div style={{
    padding: '12px',
    marginBottom: '12px',
    backgroundColor: '#f0f9ff',
    border: '1px solid #0ea5e9',
    borderRadius: '8px',
    fontSize: '14px',
    position: 'relative'
  }}>
    <div style={{
      display: 'flex',
      justifyContent: 'space-between',
      alignItems: 'center',
      marginBottom: '8px'
    }}>
      <strong style={{ color: '#0ea5e9' }}>📄 Selected Text:</strong>
      <button
        onClick={() => setSelectedText('')}
        style={{
          background: 'none',
          border: 'none',
          cursor: 'pointer',
          fontSize: '18px',
          color: '#64748b'
        }}
        aria-label="Clear selected text"
      >
        ×
      </button>
    </div>
    <div style={{
      color: '#334155',
      maxHeight: '100px',
      overflowY: 'auto',
      whiteSpace: 'pre-wrap',
      wordBreak: 'break-word'
    }}>
      {selectedText.length > 200 
        ? `${selectedText.substring(0, 200)}...` 
        : selectedText}
    </div>
  </div>
)}
```

### Step 4: Combine Selected Text with User Input

Modify your `sendMessage` function to include the selected text:

```typescript
const sendMessage = async (userInput: string) => {
  // Combine selected text with user input
  let fullMessage = userInput;
  
  if (selectedText) {
    // Format: Selected text first, then user question
    fullMessage = `Context from page:\n${selectedText}\n\nUser question: ${userInput}`;
    
    // Alternative format (if you prefer):
    // fullMessage = `${selectedText}\n\n---\n\n${userInput}`;
  }

  // Send to your API
  const response = await fetch('your-api-endpoint', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      message: fullMessage,
      // ... other data
    }),
  });

  // After sending, optionally clear selected text
  // setSelectedText('');
};
```

## Complete Example (React/TypeScript)

Here's a complete example component:

```tsx
import React, { useState, useEffect } from 'react';

export default function EmbeddedChatbot() {
  const [selectedText, setSelectedText] = useState<string>('');
  const [input, setInput] = useState('');
  const [messages, setMessages] = useState<any[]>([]);

  // Listen for messages from parent window
  useEffect(() => {
    const handleMessage = (event: MessageEvent) => {
      const allowedOrigins = [
        'http://localhost:3000',
        'http://localhost:5173',
        window.location.origin,
      ];

      if (!allowedOrigins.includes(event.origin)) return;

      if (event.data.type === 'SET_SELECTED_TEXT') {
        setSelectedText(event.data.text);
      }

      if (event.data.type === 'CLEAR_SELECTED_TEXT') {
        setSelectedText('');
      }
    };

    window.addEventListener('message', handleMessage);
    return () => window.removeEventListener('message', handleMessage);
  }, []);

  const sendMessage = async () => {
    if (!input.trim()) return;

    // Combine selected text with user input
    const fullMessage = selectedText
      ? `Context from page:\n${selectedText}\n\nUser question: ${input}`
      : input;

    // Add user message to chat
    const userMsg = { role: 'user', content: input };
    setMessages(prev => [...prev, userMsg]);

    // Send to API
    try {
      const response = await fetch('your-api-endpoint', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ message: fullMessage }),
      });

      const data = await response.json();
      setMessages(prev => [...prev, { role: 'assistant', content: data.response }]);
    } catch (error) {
      console.error('Error:', error);
    }

    setInput('');
  };

  return (
    <div style={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
      {/* Selected Text Display */}
      {selectedText && (
        <div style={{
          padding: '12px',
          margin: '12px',
          backgroundColor: '#f0f9ff',
          border: '1px solid #0ea5e9',
          borderRadius: '8px',
          fontSize: '14px'
        }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '8px' }}>
            <strong style={{ color: '#0ea5e9' }}>📄 Selected Text:</strong>
            <button onClick={() => setSelectedText('')} style={{ background: 'none', border: 'none', cursor: 'pointer' }}>
              ×
            </button>
          </div>
          <div style={{ color: '#334155', maxHeight: '100px', overflowY: 'auto' }}>
            {selectedText.length > 200 ? `${selectedText.substring(0, 200)}...` : selectedText}
          </div>
        </div>
      )}

      {/* Messages */}
      <div style={{ flex: 1, overflowY: 'auto', padding: '12px' }}>
        {messages.map((msg, idx) => (
          <div key={idx} style={{ marginBottom: '12px' }}>
            <strong>{msg.role}:</strong> {msg.content}
          </div>
        ))}
      </div>

      {/* Input */}
      <div style={{ padding: '12px', borderTop: '1px solid #e2e8f0' }}>
        <input
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
          placeholder="Type your message..."
          style={{ width: '100%', padding: '8px', borderRadius: '4px', border: '1px solid #cbd5e1' }}
        />
        <button onClick={sendMessage} style={{ marginTop: '8px', padding: '8px 16px' }}>
          Send
        </button>
      </div>
    </div>
  );
}
```

## Message Types

The parent page sends these message types:

1. **`SET_SELECTED_TEXT`**
   - `event.data.type`: `'SET_SELECTED_TEXT'`
   - `event.data.text`: The selected text string
   - `event.data.timestamp`: Timestamp when text was selected

2. **`CLEAR_SELECTED_TEXT`**
   - `event.data.type`: `'CLEAR_SELECTED_TEXT'`
   - No additional data

## Testing

1. Open your embedded chatbot at `http://localhost:5171/embed`
2. Open browser console to see messages
3. In the parent page, select some text
4. You should see the message logged in console
5. The selected text should appear in the chatbot UI
6. When you send a message, it should include the selected text

## Security Notes

- Always verify `event.origin` before processing messages
- Only accept messages from trusted origins
- Don't trust the content of messages - validate and sanitize if needed

## Implementation Notes (Current Implementation)

The current implementation uses a **header-based approach**:

1. **Frontend (Embedded Chatbot)**:
   - Receives `SET_SELECTED_TEXT` via postMessage
   - Displays text in `SelectedTextDisplay` component
   - Intercepts fetch requests in `ChatKitPanel`
   - Adds `X-Selected-Text` header (URL encoded) to API requests

2. **Backend**:
   - Extracts `X-Selected-Text` header from incoming requests
   - URL decodes the selected text
   - Prepends to user message: `"Context from page:\n{selectedText}\n\nUser question: {userMessage}"`
   - Prevents double-adding (checks if already prepended)

**Advantages of this approach**:
- Clean separation: UI context separate from message body
- Backend can handle context formatting consistently
- Works with existing API structure
- Easy to debug (headers visible in network tab)

**Alternative approach** (if needed):
- Could also modify request body directly in frontend
- Both approaches work, header-based is cleaner for backend processing

