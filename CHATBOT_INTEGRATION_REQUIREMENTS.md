# Chatbot Integration Requirements for Selected Text Feature

## Overview
The ChatWidget sends selected text from the documentation page to the chatbot iframe. The chatbot needs to:
1. **Display selected text at the TOP of the chatbot UI**
2. **Automatically append selected text to user messages when sending**

## Message Format

The ChatWidget sends messages via `postMessage` with the following structure:

### SET_SELECTED_TEXT Message
```javascript
{
  type: 'SET_SELECTED_TEXT',
  text: 'The selected text from the page',
  timestamp: 1234567890,
  instruction: 'Display this text at the top of the chatbot UI and append it to user messages when sending',
  displayAtTop: true,
  appendToMessages: true
}
```

### CLEAR_SELECTED_TEXT Message
```javascript
{
  type: 'CLEAR_SELECTED_TEXT'
}
```

## Required Implementation

### Step 1: Listen for Messages

Add this to your chatbot component:

```javascript
useEffect(() => {
  const handleMessage = (event: MessageEvent) => {
    // Security: Verify the origin
    const allowedOrigins = [
      'http://localhost:3000',
      'https://your-production-domain.com',
      window.location.origin,
    ];

    // Accept messages from parent window
    if (event.origin !== window.location.origin && !allowedOrigins.includes(event.origin)) {
      // Also accept from parent (cross-origin)
      if (event.source !== window.parent) {
        console.warn('Blocked message from unauthorized origin:', event.origin);
        return;
      }
    }

    // Handle SET_SELECTED_TEXT message
    if (event.data.type === 'SET_SELECTED_TEXT') {
      const selectedText = event.data.text;
      
      // Store in state
      setSelectedText(selectedText);
      
      // Display at top of chatbot UI
      console.log('Selected text received:', selectedText);
    }

    // Handle CLEAR_SELECTED_TEXT message
    if (event.data.type === 'CLEAR_SELECTED_TEXT') {
      setSelectedText('');
      console.log('Selected text cleared');
    }
  };

  window.addEventListener('message', handleMessage);
  return () => window.removeEventListener('message', handleMessage);
}, []);
```

### Step 2: Display Selected Text at Top of UI

Add a banner component at the top of your chatbot UI (above messages, below header):

```jsx
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

### Step 3: Append Selected Text to Messages

Modify your `sendMessage` function to automatically combine selected text with user input:

```javascript
const sendMessage = async (userInput: string) => {
  if (!userInput.trim()) return;

  // ALWAYS combine selected text with user message
  const fullMessage = selectedText 
    ? `${selectedText}\n\n---\n\nUser question: ${userInput}`
    : userInput;

  // Send to API
  const response = await fetch('your-api-endpoint', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ 
      message: fullMessage  // This includes both selected text and user input
    }),
  });

  // Handle response...
};
```

## Complete Example Structure

```jsx
function Chatbot() {
  const [selectedText, setSelectedText] = useState('');
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');

  // Listen for messages from parent
  useEffect(() => {
    const handleMessage = (event: MessageEvent) => {
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
    const fullMessage = selectedText 
      ? `${selectedText}\n\n---\n\nUser question: ${input}`
      : input;
    
    // Send fullMessage to API
    // ...
  };

  return (
    <div>
      {/* Selected Text Banner - AT THE TOP */}
      {selectedText && (
        <div className="selected-text-banner">
          <strong>📄 Selected Text:</strong>
          <p>{selectedText}</p>
          <button onClick={() => setSelectedText('')}>×</button>
        </div>
      )}

      {/* Messages */}
      <div className="messages">
        {messages.map(msg => <div key={msg.id}>{msg.content}</div>)}
      </div>

      {/* Input */}
      <input 
        value={input}
        onChange={(e) => setInput(e.target.value)}
        onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
      />
    </div>
  );
}
```

## Testing

1. Open browser console
2. Select text on the documentation page
3. Check console for: `[ChatWidget] Selected text sent to chatbot`
4. In chatbot console, check for: `Selected text received: ...`
5. Verify selected text appears at top of chatbot UI
6. Send a message and verify it includes the selected text

## Important Notes

- **Display Location**: Selected text MUST be displayed at the TOP of the chatbot UI (above messages)
- **Auto-Append**: Selected text MUST be automatically appended to every user message
- **Message Format**: Use the format: `{selectedText}\n\n---\n\nUser question: {userInput}`
- **Security**: Always verify message origins for security

