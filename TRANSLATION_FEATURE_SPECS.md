# Translation Feature - Technical Specifications

## Document Information

- **Feature Name**: Page Translation to Urdu
- **Version**: 1.0.0
- **Last Updated**: 2024
- **Status**: Production Ready
- **Framework**: Docusaurus v3+ with React 18+

---

## 1. Feature Overview

### 1.1 Purpose
Enable users to translate entire documentation pages from English to Urdu with a single click, providing an accessible reading experience for Urdu-speaking users.

### 1.2 Scope
- Client-side translation of page content
- Support for English → Urdu translation
- Automatic button injection on all documentation pages
- Original text restoration capability
- Theme-aware UI (light/dark mode)

### 1.3 Out of Scope
- Server-side translation
- Translation caching
- Multiple language support (beyond Urdu)
- Offline translation
- Code block translation
- Image/alt text translation

---

## 2. Component Specifications

### 2.1 TranslationButton Component

**File**: `src/components/TranslationButton.tsx`

#### 2.1.1 Props Interface

```typescript
interface TranslationButtonProps {
  className?: string;  // Optional CSS class name
}
```

#### 2.1.2 State Management

```typescript
// State Variables
const [isTranslating, setIsTranslating] = useState<boolean>(false);
const [isTranslated, setIsTranslated] = useState<boolean>(false);
const [originalContent, setOriginalContent] = useState<Map<HTMLElement, string>>(new Map());
const [colorMode, setColorMode] = useState<'light' | 'dark'>('light');
const [error, setError] = useState<string | null>(null);

// Refs
const contentRef = useRef<HTMLElement | null>(null);
const pageIdRef = useRef<string>('');
```

#### 2.1.3 Component Lifecycle

**Mount Behavior**:
- Detects main content area (`<article>` or `.markdown`)
- Initializes color mode detection
- Sets up MutationObservers for theme/content changes
- Tracks current page pathname

**Unmount Behavior**:
- Cleans up MutationObservers
- Removes event listeners
- Clears timers

#### 2.1.4 Methods

**`translateToUrdu()`**:
- **Type**: `async () => Promise<void>`
- **Purpose**: Main translation handler
- **Flow**:
  1. Check if already translated → restore if yes
  2. Find and collect all text elements
  3. Store original content in Map
  4. Translate elements in batches
  5. Update DOM with translated text
  6. Set translation state

**`translateText(text: string, retryCount: number)`**:
- **Type**: `async (text: string, retryCount?: number) => Promise<string>`
- **Purpose**: Translate single text string using APIs
- **Returns**: Translated text or original if translation fails
- **Retry Logic**: Up to 2 retries with exponential backoff

#### 2.1.5 UI States

| State | Button Text | Icon | Disabled |
|-------|-------------|------|----------|
| Default | "Translate to Urdu" | 🌐 | No |
| Loading | "Translating..." | ⟳ (spinning) | Yes |
| Translated | "Show Original" | 🔙 | No |
| Error | "Translate to Urdu" | 🌐 | No |

---

## 3. API Specifications

### 3.1 MyMemory Translation API (Primary)

**Base URL**: `https://api.mymemory.translated.net`

#### 3.1.1 Endpoint

```
GET /get
```

#### 3.1.2 Request Parameters

| Parameter | Type | Required | Description |
|----------|------|----------|-------------|
| `q` | string | Yes | URL-encoded text to translate |
| `langpair` | string | Yes | Language pair in format `source|target` |

**Example**:
```
GET /get?q=Hello%20World&langpair=en|ur
```

#### 3.1.3 Request Headers

```http
Accept: application/json
```

#### 3.1.4 Response Format

**Success Response (200 OK)**:
```json
{
  "responseData": {
    "translatedText": "ہیلو ورلڈ",
    "match": 1
  },
  "responseStatus": 200,
  "quotaFinished": false,
  "mtLangSupported": null,
  "responseDetails": "",
  "responderId": null,
  "exception_code": null,
  "matches": [...]
}
```

**Error Response**:
```json
{
  "responseStatus": 429,
  "responseDetails": "Rate limit exceeded"
}
```

#### 3.1.5 Rate Limits

- **Free Tier**: ~10,000 words/day
- **Request Rate**: ~100 requests/hour
- **No API Key Required**: Public endpoint

#### 3.1.6 Error Codes

| Code | Meaning | Handling |
|------|---------|----------|
| 200 | Success | Use translated text |
| 429 | Rate Limited | Retry with delay or use fallback |
| 500 | Server Error | Use fallback API |
| Network Error | Connection Failed | Retry or show error |

---

### 3.2 LibreTranslate API (Fallback)

**Base URL**: `https://libretranslate.de`

#### 3.2.1 Endpoint

```
POST /translate
```

#### 3.2.2 Request Headers

```http
Content-Type: application/json
Accept: application/json
```

#### 3.2.3 Request Body

```json
{
  "q": "text to translate",
  "source": "en",
  "target": "ur",
  "format": "text"
}
```

#### 3.2.4 Response Format

**Success Response (200 OK)**:
```json
{
  "translatedText": "ہیلو ورلڈ"
}
```

**Error Response**:
```json
{
  "error": "Invalid language code"
}
```

#### 3.2.5 Rate Limits

- **Public Instance**: Rate limited (exact limits unknown)
- **No API Key Required**: Public endpoint
- **CORS**: May have restrictions

---

## 4. Integration Specifications

### 4.1 Docusaurus Theme Swizzle

**File**: `src/theme/DocItem/Layout/index.tsx`

#### 4.1.1 Swizzle Command

```bash
npx docusaurus swizzle @docusaurus/theme-classic DocItem/Layout --wrap --typescript
```

#### 4.1.2 Integration Method

**Wrapper Pattern**:
- Wraps original `DocItem/Layout` component
- Does not modify core Docusaurus code
- Preserves original functionality

#### 4.1.3 Button Injection Logic

```typescript
// Pseudo-code
1. Wait 300ms for content to render
2. Find <article> element
3. Find <h1> within article
4. Check if button already exists
5. Create container div
6. Render TranslationButton using React.createRoot
7. Insert container before h1
```

#### 4.1.4 Timing

- **Initial Delay**: 300ms after component mount
- **Reason**: Ensures DOM is fully rendered
- **Cleanup**: Clears timeout on unmount

---

## 5. Content Selection Specifications

### 5.1 Target Elements

**Included Elements**:
- `p` - Paragraphs
- `h1, h2, h3, h4, h5, h6` - Headings
- `li` - List items
- `blockquote` - Blockquotes
- `td, th` - Table cells
- `span` - Spans (excluding code-related)

**Excluded Elements**:
- Elements inside `<pre>` tags
- Elements inside `<code>` tags
- Elements inside `<nav>` tags
- Elements inside `<button>` tags
- Elements inside `.navbar`
- Empty text nodes
- Elements with text length < 3 characters
- Elements with text length > 500 characters

### 5.2 Content Storage

**Data Structure**: `Map<HTMLElement, string>`

**Purpose**: 
- Store original text for each element
- Enable restoration of original content
- Maintain element reference for DOM updates

**Example**:
```typescript
Map {
  <p> => "Hello World",
  <h1> => "Chapter Title",
  <li> => "List item"
}
```

---

## 6. Translation Processing Specifications

### 6.1 Batch Processing

**Batch Size**: 2 elements per batch

**Processing Method**: Sequential within batches

**Delay Between Requests**: 500ms

**Rationale**: 
- Prevents API rate limiting
- Reduces server load
- Improves success rate

### 6.2 Retry Logic

**Max Retries**: 2 per API call

**Backoff Strategy**: Exponential
- Retry 1: 1 second delay
- Retry 2: 2 second delay

**Retry Conditions**:
- Network errors
- API failures (non-200 status)
- Empty/invalid responses

### 6.3 Fallback Strategy

**Priority Order**:
1. MyMemory API (primary)
2. LibreTranslate API (fallback)
3. Original text (if all fail)

**Fallback Triggers**:
- HTTP error status
- Network timeout
- Invalid response format
- Empty translation result

---

## 7. Error Handling Specifications

### 7.1 Error Types

| Error Type | Cause | User Feedback | Recovery |
|------------|-------|---------------|----------|
| API Failure | Network/Server error | Error message | Retry with fallback API |
| Rate Limit | Too many requests | Error message | Wait and retry |
| Empty Translation | API returned original | Warning log | Continue with next element |
| Content Not Found | Article element missing | Silent fail | No action |
| Invalid Response | Malformed JSON | Error message | Use fallback API |

### 7.2 Error Display

**Location**: Below translation button

**Styling**:
- Light mode: Red background (#fee2e2), dark text (#991b1b)
- Dark mode: Red background (#dc2626), light text (#ffffff)
- Position: Absolute, below button
- Max width: 300px
- Auto-dismiss: On next translation attempt

### 7.3 Error Logging

**Console Logs**:
- Translation start: `Starting translation of X elements...`
- Progress: `Translation progress: X/Y (Z failed)`
- Completion: `Translation complete: X elements translated`
- Errors: `Translation error: [error details]`

---

## 8. State Management Specifications

### 8.1 State Variables

| Variable | Type | Purpose | Reset Condition |
|----------|------|---------|-----------------|
| `isTranslating` | boolean | Loading state | Translation complete/failed |
| `isTranslated` | boolean | Translation status | Page navigation, restore click |
| `originalContent` | Map | Original text storage | Page navigation |
| `colorMode` | 'light'\|'dark' | Theme detection | Theme change |
| `error` | string\|null | Error messages | New translation attempt |

### 8.2 State Reset Triggers

1. **Page Navigation**: Detected via pathname change
2. **Content Update**: Detected via MutationObserver
3. **Manual Reset**: "Show Original" button click

### 8.3 State Persistence

**Not Persisted**:
- Translation state (resets on page change)
- Original content (resets on navigation)
- Error messages (cleared on new attempt)

**Reason**: Each page should start fresh

---

## 9. Performance Specifications

### 9.1 Translation Speed

**Estimated Time**:
- Small page (10-20 elements): ~10-15 seconds
- Medium page (30-50 elements): ~25-40 seconds
- Large page (50+ elements): ~50+ seconds

**Factors**:
- Number of elements
- API response time
- Network latency
- Rate limiting delays

### 9.2 Resource Usage

**Memory**:
- Original content Map: ~1-5 MB (depending on page size)
- React state: Minimal

**Network**:
- ~1 request per text element
- Average request size: < 1 KB
- Total for medium page: ~30-50 KB

**CPU**:
- Minimal (mostly I/O bound)
- DOM manipulation: Negligible

### 9.3 Optimization Strategies

1. **Batch Processing**: Reduces concurrent requests
2. **Sequential Processing**: Prevents API overload
3. **Request Delays**: Respects rate limits
4. **Element Filtering**: Skips unnecessary elements
5. **Early Validation**: Checks text length before API call

---

## 10. UI/UX Specifications

### 10.1 Button Appearance

**Default State**:
- Background: White (light) / Dark gray (dark)
- Border: Light gray / Medium gray
- Text: "Translate to Urdu"
- Icon: 🌐
- Padding: 8px 16px
- Border radius: 8px

**Hover State**:
- Background: Light gray / Medium gray
- Transform: TranslateY(-1px)
- Shadow: Enhanced

**Loading State**:
- Cursor: Wait
- Icon: Spinning ⟳
- Text: "Translating..."
- Disabled: Yes

**Translated State**:
- Icon: 🔙
- Text: "Show Original"

### 10.2 Button Position

**Location**: Above chapter title (h1)

**Container**:
- Margin bottom: 1.5rem
- Display: Flex
- Justify content: Flex-start
- Align items: Center

### 10.3 Responsive Behavior

**Mobile**: Button remains same size, may wrap if needed

**Tablet**: Same as desktop

**Desktop**: Full button visible

---

## 11. Browser Compatibility

### 11.1 Required Features

- **Fetch API**: For HTTP requests
- **Promise/async-await**: For asynchronous operations
- **Map**: For storing original content
- **MutationObserver**: For content change detection
- **React 18+**: For createRoot API

### 11.2 Supported Browsers

| Browser | Minimum Version | Notes |
|---------|----------------|-------|
| Chrome | 90+ | Full support |
| Firefox | 88+ | Full support |
| Safari | 14+ | Full support |
| Edge | 90+ | Full support |

### 11.3 Polyfills

**Not Required**: All features are natively supported in modern browsers

---

## 12. Security Specifications

### 12.1 Data Privacy

**What is Sent**:
- Page text content to translation APIs
- No user identification
- No personal data

**Considerations**:
- Content is sent to third-party APIs
- Sensitive content should not be translated
- No data is stored by our application

### 12.2 API Security

**No Authentication Required**: Using public free APIs

**CORS**: APIs must support browser CORS requests

**HTTPS**: All API calls use HTTPS

### 12.3 Content Security

**XSS Protection**: 
- Uses `textContent` (not `innerHTML`)
- No user input is directly rendered
- React escapes content automatically

---

## 13. Testing Specifications

### 13.1 Manual Testing Checklist

- [ ] Button appears on all doc pages
- [ ] Button is positioned above h1 title
- [ ] Translation works for small pages
- [ ] Translation works for large pages
- [ ] "Show Original" restores text
- [ ] Error handling works (disconnect network)
- [ ] Theme switching works (light/dark)
- [ ] Page navigation resets state
- [ ] Console logs show progress
- [ ] No console errors

### 13.2 Test Scenarios

**Scenario 1: Normal Translation**
1. Navigate to doc page
2. Click "Translate to Urdu"
3. Wait for completion
4. Verify text is translated
5. Click "Show Original"
6. Verify text is restored

**Scenario 2: Error Handling**
1. Disconnect network
2. Click translate
3. Verify error message appears
4. Reconnect network
5. Retry translation
6. Verify success

**Scenario 3: Page Navigation**
1. Translate page A
2. Navigate to page B
3. Verify page B is not translated
4. Translate page B
5. Verify translation works

---

## 14. Deployment Specifications

### 14.1 Build Requirements

**No Additional Build Steps**: Component is included in standard Docusaurus build

**Dependencies**: 
- React 18+
- Docusaurus 3+
- No external translation libraries

### 14.2 Environment Variables

**Not Required**: Uses public APIs with no authentication

### 14.3 Production Considerations

1. **Rate Limiting**: Monitor API usage
2. **Error Monitoring**: Track translation failures
3. **Performance**: Monitor translation times
4. **User Feedback**: Collect usage statistics

---

## 15. Maintenance Specifications

### 15.1 API Monitoring

**What to Monitor**:
- API response times
- Error rates
- Rate limit hits
- Translation quality

### 15.2 Update Procedures

**API Changes**:
- Update endpoint URLs if APIs change
- Update request/response handling if formats change
- Test fallback mechanisms

**Component Updates**:
- Follow React best practices
- Maintain backward compatibility
- Test on all supported browsers

### 15.3 Troubleshooting

**Common Issues**:
1. **Translation not working**: Check API status, network connection
2. **Rate limiting**: Increase delays, reduce batch size
3. **Button not appearing**: Check DOM structure, timing
4. **State not resetting**: Verify page navigation detection

---

## 16. Future Enhancements

### 16.1 Planned Features

- [ ] Translation caching (localStorage)
- [ ] Progress bar UI
- [ ] Multiple language support
- [ ] Batch API requests (if supported)
- [ ] Offline translation (service worker)

### 16.2 Potential Improvements

- [ ] Paid API integration (Google Translate)
- [ ] Translation quality selection
- [ ] User preference storage
- [ ] Analytics integration
- [ ] A/B testing for API selection

---

## 17. API Reference Summary

### 17.1 MyMemory API

```
GET https://api.mymemory.translated.net/get
Query Params:
  - q: string (URL-encoded)
  - langpair: "en|ur"
Response: { responseData: { translatedText: string } }
```

### 17.2 LibreTranslate API

```
POST https://libretranslate.de/translate
Headers: Content-Type: application/json
Body: { q: string, source: "en", target: "ur", format: "text" }
Response: { translatedText: string }
```

---

## 18. Code Examples

### 18.1 Basic Usage

```typescript
// Component is automatically injected
// No manual usage required
// Button appears on all doc pages
```

### 18.2 Custom Integration

```typescript
import TranslationButton from '@site/src/components/TranslationButton';

function CustomPage() {
  return (
    <div>
      <TranslationButton />
      <h1>My Page</h1>
      <p>Content here...</p>
    </div>
  );
}
```

### 18.3 API Call Example

```typescript
// MyMemory API
const response = await fetch(
  `https://api.mymemory.translated.net/get?q=${encodeURIComponent(text)}&langpair=en|ur`
);
const data = await response.json();
const translated = data.responseData.translatedText;
```

---

## 19. Glossary

- **API**: Application Programming Interface
- **CORS**: Cross-Origin Resource Sharing
- **DOM**: Document Object Model
- **Rate Limiting**: Restrictions on API request frequency
- **Swizzle**: Docusaurus theme customization method
- **MutationObserver**: Browser API for watching DOM changes

---

## 20. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2024 | Initial release with MyMemory and LibreTranslate APIs |

---

## Appendix A: File Structure

```
src/
├── components/
│   └── TranslationButton.tsx      # Main component
└── theme/
    └── DocItem/
        └── Layout/
            └── index.tsx          # Integration wrapper
```

## Appendix B: Dependencies

```json
{
  "react": "^18.0.0",
  "@docusaurus/core": "^3.0.0",
  "@docusaurus/theme-classic": "^3.0.0"
}
```

---

**End of Specifications**


