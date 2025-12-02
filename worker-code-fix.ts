/**
 * Cloudflare Worker - Translation Service
 * 
 * This is the COMPLETE worker code that handles CORS properly.
 * Copy this entire file to your worker's src/index.ts (or main file)
 */

// CORS headers - allow requests from any origin (change '*' to specific domain for production)
const corsHeaders = {
  'Access-Control-Allow-Origin': '*', // Change to 'http://localhost:3000' for dev, or your production domain
  'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
  'Access-Control-Allow-Headers': 'Content-Type',
  'Access-Control-Max-Age': '86400', // 24 hours
};

// Your translation function - replace this with your actual translation logic
async function translateText(text: string): Promise<string> {
  // TODO: Replace with your actual translation API call
  // Example: Call Google Translate, DeepL, or your translation service
  
  // For now, this is a placeholder - you need to implement your translation logic here
  // Example:
  // const response = await fetch('YOUR_TRANSLATION_API_URL', { ... });
  // return translatedText;
  
  return `[Translated] ${text}`; // Placeholder - replace this!
}

export default {
  async fetch(request: Request): Promise<Response> {
    const url = new URL(request.url);
    
    // ============================================
    // CRITICAL: Handle CORS preflight (OPTIONS)
    // ============================================
    if (request.method === 'OPTIONS') {
      return new Response(null, {
        status: 204, // No Content - this is the correct status for OPTIONS
        headers: corsHeaders,
      });
    }
    
    // ============================================
    // Handle POST requests for translation
    // ============================================
    if (request.method === 'POST') {
      try {
        // Parse request body
        const body = await request.json();
        
        // Single text translation
        if (body.text && typeof body.text === 'string') {
          const translation = await translateText(body.text);
          
          return new Response(
            JSON.stringify({ translation }),
            {
              status: 200,
              headers: {
                'Content-Type': 'application/json',
                ...corsHeaders, // Include CORS headers in response
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
                ...corsHeaders, // Include CORS headers in response
              },
            }
          );
        }
        
        // Invalid request format
        return new Response(
          JSON.stringify({ 
            error: 'Invalid request. Expected { text: string } or { batch: string[] }' 
          }),
          {
            status: 400,
            headers: {
              'Content-Type': 'application/json',
              ...corsHeaders,
            },
          }
        );
      } catch (error) {
        // Error handling
        const errorMessage = error instanceof Error ? error.message : 'Unknown error';
        
        return new Response(
          JSON.stringify({ 
            error: errorMessage 
          }),
          {
            status: 500,
            headers: {
              'Content-Type': 'application/json',
              ...corsHeaders, // Include CORS headers even in error responses
            },
          }
        );
      }
    }
    
    // ============================================
    // Handle GET requests (optional - for health check)
    // ============================================
    if (request.method === 'GET') {
      return new Response(
        JSON.stringify({ 
          status: 'ok', 
          service: 'translation-worker',
          endpoints: {
            single: 'POST with { text: string }',
            batch: 'POST with { batch: string[] }'
          }
        }),
        {
          status: 200,
          headers: {
            'Content-Type': 'application/json',
            ...corsHeaders,
          },
        }
      );
    }
    
    // ============================================
    // Method not allowed
    // ============================================
    return new Response(
      JSON.stringify({ 
        error: 'Method not allowed. Use POST or OPTIONS.' 
      }),
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


