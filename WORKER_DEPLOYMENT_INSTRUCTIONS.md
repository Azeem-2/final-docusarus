# Cloudflare Worker Deployment Instructions

## Quick Fix for CORS Error

The worker is currently returning HTTP 500 on OPTIONS requests. Follow these steps to fix it.

## Step 1: Update Worker Code

1. **Navigate to your worker directory**:
   ```bash
   cd g:/translate/my-worker
   ```

2. **Open the main worker file** (usually `src/index.ts` or `index.ts`)

3. **Replace the entire file content** with the code from `worker-code-fix.ts`

   OR manually add this at the **very beginning** of your fetch handler:

   ```typescript
   // Add CORS headers constant at the top
   const corsHeaders = {
     'Access-Control-Allow-Origin': '*',
     'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
     'Access-Control-Allow-Headers': 'Content-Type',
     'Access-Control-Max-Age': '86400',
   };

   export default {
     async fetch(request: Request): Promise<Response> {
       // ===== ADD THIS FIRST =====
       if (request.method === 'OPTIONS') {
         return new Response(null, {
           status: 204,
           headers: corsHeaders,
         });
       }
       // ===== END OF CORS FIX =====
       
       // ... rest of your code
       
       // IMPORTANT: Add corsHeaders to ALL responses
       return new Response(JSON.stringify({ ... }), {
         status: 200,
         headers: {
           'Content-Type': 'application/json',
           ...corsHeaders, // <-- Add this to every response
         },
       });
     },
   };
   ```

## Step 2: Deploy the Worker

```bash
cd g:/translate/my-worker
npx wrangler deploy
```

## Step 3: Test the Fix

### Test 1: CORS Preflight (OPTIONS)
```bash
curl -X OPTIONS https://my-worker.translate-worker.workers.dev \
  -H "Origin: http://localhost:3000" \
  -H "Access-Control-Request-Method: POST" \
  -H "Access-Control-Request-Headers: Content-Type" \
  -v
```

**Expected Result**: `204 No Content` with CORS headers

### Test 2: Single Translation (POST)
```bash
curl -X POST https://my-worker.translate-worker.workers.dev \
  -H "Content-Type: application/json" \
  -H "Origin: http://localhost:3000" \
  -d '{"text": "Hello, how are you?"}' \
  -v
```

**Expected Result**: `200 OK` with `{"translation": "..."}`

### Test 3: Batch Translation (POST)
```bash
curl -X POST https://my-worker.translate-worker.workers.dev \
  -H "Content-Type: application/json" \
  -H "Origin: http://localhost:3000" \
  -d '{"batch": ["Hello", "World", "How are you?"]}' \
  -v
```

**Expected Result**: `200 OK` with `{"translations": ["...", "...", "..."]}`

## Step 4: Test in Browser

1. Open your Docusaurus site: `http://localhost:3000`
2. Navigate to any documentation page
3. Click "Translate to Urdu" button
4. Check browser console - should see translation progress
5. Verify text is translated

## Common Issues

### Issue: Still getting 500 error

**Check**:
- Did you add the OPTIONS handler?
- Is it at the **very beginning** of the fetch handler?
- Did you deploy the updated worker?

**Fix**:
```typescript
// Make sure this is the FIRST thing in your fetch handler
if (request.method === 'OPTIONS') {
  return new Response(null, {
    status: 204,
    headers: corsHeaders,
  });
}
```

### Issue: CORS headers not in response

**Check**:
- Did you add `...corsHeaders` to ALL response headers?
- Check Network tab in browser - look for `Access-Control-Allow-Origin` header

**Fix**:
```typescript
// Every response must include CORS headers
return new Response(data, {
  headers: {
    'Content-Type': 'application/json',
    ...corsHeaders, // <-- Must be in every response
  },
});
```

### Issue: Worker code has syntax errors

**Check**:
- Run `npx wrangler dev` to test locally first
- Check for TypeScript errors

**Fix**:
- Use the complete code from `worker-code-fix.ts`
- Make sure all imports are correct

## Security: Restrict Origins (Production)

For production, restrict CORS to specific origins:

```typescript
const allowedOrigins = [
  'http://localhost:3000',
  'https://ai-native.panaversity.org',
  // Add your production domains
];

const origin = request.headers.get('Origin');

const corsHeaders = {
  'Access-Control-Allow-Origin': allowedOrigins.includes(origin || '') 
    ? origin || '*' 
    : 'null',
  'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
  'Access-Control-Allow-Headers': 'Content-Type',
  'Access-Control-Max-Age': '86400',
};
```

## Verification Checklist

After deploying, verify:

- [ ] OPTIONS request returns 204 with CORS headers
- [ ] POST request with `{ text: "..." }` returns translation
- [ ] POST request with `{ batch: [...] }` returns translations array
- [ ] All responses include CORS headers
- [ ] Browser console shows no CORS errors
- [ ] Translation button works in Docusaurus

## Need Help?

If you're still getting errors:

1. **Check worker logs** in Cloudflare Dashboard
2. **Test with curl** (commands above)
3. **Verify worker code** matches `worker-code-fix.ts`
4. **Check deployment** - make sure latest code is deployed

The key fix is: **Handle OPTIONS requests FIRST, before any other logic!**


