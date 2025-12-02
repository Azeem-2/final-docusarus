import React from "react";
import Layout from "@theme/Layout";
import ProtectedRoute from "@site/src/components/ProtectedRoute";
import { useAuth } from "@site/src/components/AuthProvider";
import { authClient } from "@site/src/lib/auth-client";

export default function ProtectedExample() {
  const { session, isAuthenticated } = useAuth();

  const handleSignOut = async () => {
    await authClient.signOut();
    if (typeof window !== "undefined") {
      window.location.href = "/";
    }
  };

  return (
    <ProtectedRoute>
      <Layout
        title="Protected Content"
        description="Example of protected content"
      >
        <div
          style={{
            maxWidth: "800px",
            margin: "0 auto",
            padding: "2rem",
          }}
        >
          <h1>🔒 Protected Content</h1>
          <p>This page is only accessible to authenticated users.</p>

          {isAuthenticated && session?.user && (
            <div
              style={{
                marginTop: "2rem",
                padding: "1.5rem",
                background: "var(--ifm-background-surface-color)",
                borderRadius: "8px",
                border: "1px solid var(--ifm-color-emphasis-300)",
              }}
            >
              <h2>User Information</h2>
              <p>
                <strong>Email:</strong> {session.user.email}
              </p>
              <p>
                <strong>Name:</strong> {session.user.name || "Not provided"}
              </p>
            </div>
          )}

          <div style={{ marginTop: "2rem" }}>
            <button
              onClick={handleSignOut}
              className="button button--danger"
            >
              Sign Out
            </button>
          </div>
        </div>
      </Layout>
    </ProtectedRoute>
  );
}


