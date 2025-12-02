# Cloudflare Worker CORS Fix Guide

## Problem

The worker is returning **HTTP 500** on OPTIONS requests (CORS preflight). This happens because:
- Browser sends OPTIONS request before POST (CORS preflight)
- Worker doesn't handle OPTIONS method
- Worker returns 500 error instead of proper CORS headers

## Solution

The worker needs to handle OPTIONS requests and return proper CORS headers.

## Worker Code Fix

Add this to your Cloudflare Worker (`src/index.ts` or main worker file):

```typescript
// CORS headers helper
const corsHeaders = {
  'Access-Control-Allow-Origin': '*', // Or specific origin: 'http://localhost:3000'
  'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
  'Access-Control-Allow-Headers': 'Content-Type',
  'Access-Control-Max-Age': '86400', // 24 hours
};

export default {
  async fetch(request: Request): Promise<Response> {
    // Handle CORS preflight (OPTIONS request)
    if (request.method === 'OPTIONS') {
      return new Response(null, {
        status: 204, // No Content
        headers: corsHeaders,
      });
    }

    // Handle actual requests (POST)
    if (request.method === 'POST') {
      try {
        const body = await request.json();
        
        // Single text translation
        if (body.text) {
          const translated = await translateText(body.text);
          return new Response(
            JSON.stringify({ translation: translated }),
            {
              status: 200,
              headers: {
                'Content-Type': 'application/json',
                ...corsHeaders,
              },
            }
          );
        }
        
        // Batch translation
        if (body.batch && Array.isArray(body.batch)) {
          const translations = await Promise.all(
            body.batch.map(text => translateText(text))
          );
          return new Response(
            JSON.stringify({ translations }),
            {
              status: 200,
              headers: {
                'Content-Type': 'application/json',
                ...corsHeaders,
              },
            }
          );
        }
        
        return new Response(
          JSON.stringify({ error: 'Invalid request body' }),
          {
            status: 400,
            headers: {
              'Content-Type': 'application/json',
              ...corsHeaders,
            },
          }
        );
      } catch (error) {
        return new Response(
          JSON.stringify({ error: error.message }),
          {
            status: 500,
            headers: {
              'Content-Type': 'application/json',
              ...corsHeaders,
            },
          }
        );
      }
    }
    
    // Method not allowed
    return new Response('Method not allowed', {
      status: 405,
      headers: corsHeaders,
    });
  },
};
```

## Complete Example Worker

Here's a complete example with translation logic:

```typescript
// src/index.ts

// CORS headers
const corsHeaders = {
  'Access-Control-Allow-Origin': '*',
  'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
  'Access-Control-Allow-Headers': 'Content-Type',
  'Access-Control-Max-Age': '86400',
};

// Translation function (replace with your actual translation logic)
async function translateText(text: string): Promise<string> {
  // Your translation API call here
  // Example: Call Google Translate API, DeepL, etc.
  // For now, returning placeholder
  return `[Translated] ${text}`;
}

export default {
  async fetch(request: Request): Promise<Response> {
    const url = new URL(request.url);
    
    // Handle CORS preflight
    if (request.method === 'OPTIONS') {
      return new Response(null, {
        status: 204,
        headers: corsHeaders,
      });
    }
    
    // Handle POST requests
    if (request.method === 'POST') {
      try {
        const body = await request.json();
        
        // Single translation
        if (body.text && typeof body.text === 'string') {
          const translation = await translateText(body.text);
          return new Response(
            JSON.stringify({ translation }),
            {
              status: 200,
              headers: {
                'Content-Type': 'application/json',
                ...corsHeaders,
              },
            }
          );
        }
        
        // Batch translation
        if (body.batch && Array.isArray(body.batch)) {
          const translations = await Promise.all(
            body.batch.map((text: string) => translateText(text))
          );
          return new Response(
            JSON.stringify({ translations }),
            {
              status: 200,
              headers: {
                'Content-Type': 'application/json',
                ...corsHeaders,
              },
            }
          );
        }
        
        // Invalid request
        return new Response(
          JSON.stringify({ error: 'Invalid request. Expected { text: string } or { batch: string[] }' }),
          {
            status: 400,
            headers: {
              'Content-Type': 'application/json',
              ...corsHeaders,
            },
          }
        );
      } catch (error) {
        return new Response(
          JSON.stringify({ 
            error: error instanceof Error ? error.message : 'Unknown error' 
          }),
          {
            status: 500,
            headers: {
              'Content-Type': 'application/json',
              ...corsHeaders,
            },
          }
        );
      }
    }
    
    // Method not allowed
    return new Response(
      JSON.stringify({ error: 'Method not allowed. Use POST or OPTIONS.' }),
      {
        status: 405,
        headers: {
          'Content-Type': 'application/json',
          ...corsHeaders,
        },
      }
    );
  },
};
```

## Security: Restrict Origins (Optional but Recommended)

For production, restrict CORS to specific origins:

```typescript
// Allowed origins
const allowedOrigins = [
  'http://localhost:3000',
  'https://ai-native.panaversity.org',
  // Add your production domain
];

// Get origin from request
const origin = request.headers.get('Origin');

// Check if origin is allowed
const corsHeaders = {
  'Access-Control-Allow-Origin': allowedOrigins.includes(origin || '') 
    ? origin || '*' 
    : 'null',
  'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
  'Access-Control-Allow-Headers': 'Content-Type',
  'Access-Control-Max-Age': '86400',
};
```

## Testing the Fix

After updating the worker:

1. **Deploy the worker**:
   ```bash
   cd g:/translate/my-worker
   npx wrangler deploy
   ```

2. **Test CORS preflight**:
   ```bash
   curl -X OPTIONS https://my-worker.translate-worker.workers.dev \
     -H "Origin: http://localhost:3000" \
     -H "Access-Control-Request-Method: POST" \
     -H "Access-Control-Request-Headers: Content-Type" \
     -v
   ```

   Should return: `204 No Content` with CORS headers

3. **Test actual request**:
   ```bash
   curl -X POST https://my-worker.translate-worker.workers.dev \
     -H "Content-Type: application/json" \
     -H "Origin: http://localhost:3000" \
     -d '{"text": "Hello, how are you?"}' \
     -v
   ```

   Should return: `200 OK` with `{"translation": "..."}`

## Quick Fix Checklist

- [ ] Add OPTIONS handler in worker
- [ ] Return 204 status for OPTIONS
- [ ] Include CORS headers in all responses
- [ ] Deploy updated worker
- [ ] Test from browser (should work now)

## Common Issues

### Issue 1: Still getting 500 error
- **Check**: Worker logs in Cloudflare dashboard
- **Fix**: Ensure OPTIONS handler is at the top of fetch handler

### Issue 2: CORS headers not working
- **Check**: Response headers in browser Network tab
- **Fix**: Make sure `Access-Control-Allow-Origin` header is present

### Issue 3: Preflight succeeds but POST fails
- **Check**: POST handler logic
- **Fix**: Ensure POST handler also includes CORS headers

## After Fix

Once the worker is fixed:
1. The OPTIONS request will return 204 with CORS headers
2. The browser will proceed with the POST request
3. Translation will work from the Docusaurus site

The client-side code in `TranslationButton.tsx` doesn't need any changes - it's already correct!


