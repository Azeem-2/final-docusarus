import React, { ReactNode, useEffect, useState, useCallback } from "react";
import Layout from "@theme/Layout";
import { useAuth } from "@site/src/components/AuthProvider";
import { authClient } from "@site/src/lib/auth-client";

interface ProtectedRouteProps {
  children: ReactNode;
  redirectTo?: string;
  fallback?: ReactNode;
  requireAuth?: boolean; // Default true, can be set to false for optional auth
  onAuthCheck?: (isAuthenticated: boolean) => void;
}

export default function ProtectedRoute({
  children,
  redirectTo = "/login",
  fallback,
  requireAuth = true,
  onAuthCheck,
}: ProtectedRouteProps) {
  const { isAuthenticated, isLoading } = useAuth();
  const [isRedirecting, setIsRedirecting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [retryCount, setRetryCount] = useState(0);
  const maxRetries = 2;

  // Validate session and handle errors
  const validateSession = useCallback(async () => {
    if (!isLoading && !isAuthenticated && requireAuth) {
      // Try to refresh session if we have retries left
      if (retryCount < maxRetries) {
        try {
          setRetryCount((prev) => prev + 1);
          // Force session refresh
          const result = await authClient.getSession();
          if (result?.data?.session) {
            // Session found, reload page to update auth state
            window.location.reload();
            return;
          }
        } catch (err) {
          console.error("Session validation error:", err);
        }
      }

      // Save current URL for redirect after login
      const currentPath = window.location.pathname + window.location.search;
      if (currentPath !== redirectTo && currentPath !== "/login") {
        sessionStorage.setItem("redirectAfterLogin", currentPath);
      }

      setIsRedirecting(true);
      // Use replace to avoid adding to history
      if (typeof window !== "undefined") {
        window.location.replace(redirectTo);
      }
    }
  }, [isAuthenticated, isLoading, requireAuth, redirectTo, retryCount]);

  // Handle auth check callback
  useEffect(() => {
    if (!isLoading && onAuthCheck) {
      onAuthCheck(isAuthenticated);
    }
  }, [isAuthenticated, isLoading, onAuthCheck]);

  // Validate and redirect if needed
  useEffect(() => {
    validateSession();
  }, [validateSession]);

  // Reset error when auth state changes
  useEffect(() => {
    if (isAuthenticated) {
      setError(null);
      setRetryCount(0);
    }
  }, [isAuthenticated]);

  // Loading state
  if (isLoading) {
    return (
      <Layout>
        <div
          style={{
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            minHeight: "50vh",
            flexDirection: "column",
            gap: "1rem",
          }}
        >
          <div
            style={{
              width: "40px",
              height: "40px",
              border: "4px solid var(--ifm-color-emphasis-300)",
              borderTop: "4px solid var(--ifm-color-primary)",
              borderRadius: "50%",
              animation: "spin 1s linear infinite",
            }}
          />
          <p style={{ color: "var(--ifm-color-content-secondary)" }}>
            Verifying authentication...
          </p>
        </div>
        <style>{`
          @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
          }
        `}</style>
      </Layout>
    );
  }

  // Redirecting state
  if (isRedirecting) {
    return (
      <Layout>
        <div
          style={{
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            minHeight: "50vh",
            flexDirection: "column",
            gap: "1rem",
          }}
        >
          <p>Redirecting to login...</p>
        </div>
      </Layout>
    );
  }

  // Error state
  if (error) {
    return (
      <Layout>
        <div
          style={{
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            minHeight: "50vh",
            flexDirection: "column",
            gap: "1rem",
            padding: "2rem",
          }}
        >
          <div
            style={{
              padding: "1rem",
              background: "var(--ifm-color-danger-lightest)",
              border: "1px solid var(--ifm-color-danger)",
              borderRadius: "8px",
              color: "var(--ifm-color-danger-darkest)",
              maxWidth: "500px",
              textAlign: "center",
            }}
          >
            <h2>Authentication Error</h2>
            <p>{error}</p>
            <button
              onClick={() => {
                setError(null);
                setRetryCount(0);
                window.location.reload();
              }}
              className="button button--primary"
              style={{ marginTop: "1rem" }}
            >
              Retry
            </button>
          </div>
        </div>
      </Layout>
    );
  }

  // Not authenticated state
  if (!isAuthenticated) {
    if (!requireAuth) {
      // Optional auth - render children anyway
      return <>{children}</>;
    }

    return (
      <Layout>
        {fallback || (
          <div
            style={{
              display: "flex",
              justifyContent: "center",
              alignItems: "center",
              minHeight: "50vh",
              flexDirection: "column",
              gap: "1rem",
              padding: "2rem",
            }}
          >
            <div
              style={{
                maxWidth: "500px",
                textAlign: "center",
                padding: "2rem",
                background: "var(--ifm-background-surface-color)",
                borderRadius: "8px",
                border: "1px solid var(--ifm-color-emphasis-300)",
              }}
            >
              <h2>🔒 Authentication Required</h2>
              <p
                style={{
                  marginBottom: "1.5rem",
                  color: "var(--ifm-color-content-secondary)",
                }}
              >
                You need to be logged in to access this content.
              </p>
              <a
                href="/login"
                className="button button--primary button--lg"
                onClick={() => {
                  // Save current URL for redirect after login
                  const currentPath =
                    window.location.pathname + window.location.search;
                  if (currentPath !== "/login") {
                    sessionStorage.setItem(
                      "redirectAfterLogin",
                      currentPath
                    );
                  }
                }}
              >
                Go to Login
              </a>
            </div>
          </div>
        )}
      </Layout>
    );
  }

  // Authenticated - render children
  return <>{children}</>;
}


