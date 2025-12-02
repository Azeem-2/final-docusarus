import { createAuthClient } from "better-auth/react";

/**
 * Better Auth Backend URL Configuration
 * 
 * Production: https://better-auth-try.vercel.app
 * 
 * To override at runtime (for development/testing):
 *   window.__BETTER_AUTH_URL__ = "http://localhost:3000";
 *   Then refresh the page.
 */
const getAuthURL = (): string => {
  if (typeof window === "undefined") {
    // SSR: default to production
    return "https://better-auth-try.vercel.app";
  }

  // Runtime override (highest priority) - allows dynamic override in browser console
  if ((window as any).__BETTER_AUTH_URL__) {
    return (window as any).__BETTER_AUTH_URL__;
  }

  // Production URL (default)
  const productionUrl = "https://better-auth-try.vercel.app";
  
  // Development detection - use localhost only if explicitly on localhost
  const hostname = window.location.hostname;
  if (hostname === "localhost" || hostname === "127.0.0.1") {
    // For local development, you can override via:
    // window.__BETTER_AUTH_URL__ = "http://localhost:3000";
    // Or just use production URL for local testing
    return productionUrl; // Default to production even on localhost
  }

  // Default to production
  return productionUrl;
};

const BETTER_AUTH_URL = getAuthURL();

// Log the auth URL in development mode for debugging
if (typeof window !== "undefined" && process.env.NODE_ENV === "development") {
  console.log("[Better Auth] Using backend URL:", BETTER_AUTH_URL);
  console.log(
    "[Better Auth] To override, set: window.__BETTER_AUTH_URL__ = 'your-url'"
  );
}

export const authClient = createAuthClient({
  baseURL: BETTER_AUTH_URL,
  fetchOptions: {
    credentials: "include", // Required for cross-origin cookies
    // Backend is configured with defaultCookieAttributes:
    // - sameSite: "none"
    // - secure: true
    // - partitioned: true
  },
});

export const { signIn, signUp, signOut, useSession } = authClient;


