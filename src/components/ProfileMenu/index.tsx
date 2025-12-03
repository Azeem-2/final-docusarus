import React, { useState } from "react";
import { useAuth } from "@site/src/components/AuthProvider";
import { authClient } from "@site/src/lib/auth-client";
import { useHistory } from "@docusaurus/router";
import AuthModal from "@site/src/components/AuthModal";
import { useToast } from "@site/src/components/Toast";

export default function ProfileMenu() {
  const { session, isAuthenticated, isLoading } = useAuth();
  const history = useHistory();
  const { showToast } = useToast();
  const [open, setOpen] = useState(false);
  const [authModalOpen, setAuthModalOpen] = useState(false);
  const [authModalMode, setAuthModalMode] =
    useState<"signin" | "signup">("signin");
  const [signingOut, setSigningOut] = useState(false);

  if (isLoading) return null;

  const toggleMenu = () => setOpen((prev) => !prev);

  const handleSignOut = async () => {
    setSigningOut(true);
    try {
      await authClient.signOut();
      setOpen(false);
      if (typeof window !== "undefined") {
        window.location.href = "/";
      } else {
        history.push("/");
      }
    } catch (err) {
      console.error("Docusaurus sign out error:", err);
    } finally {
      setSigningOut(false);
    }
  };

  const initial =
    session?.user?.name?.[0] ||
    session?.user?.email?.[0] ||
    session?.session?.user?.name?.[0] ||
    session?.session?.user?.email?.[0] ||
    "?";

  return (
    <div
      style={{
        display: "flex",
        alignItems: "center",
        gap: "0.5rem",
        marginLeft: "0.75rem",
      }}
    >
      {isAuthenticated ? (
        <div style={{ position: "relative" }}>
          <button
            type="button"
            onClick={toggleMenu}
            style={{
              width: "2.75rem",
              height: "2.75rem",
              borderRadius: "50%",
              border: "2px solid transparent",
              background: "linear-gradient(135deg, var(--ifm-color-primary), var(--ifm-color-primary-dark))",
              display: "flex",
              alignItems: "center",
              justifyContent: "center",
              fontWeight: 700,
              fontSize: "1rem",
              color: "#ffffff",
              cursor: "pointer",
              transition: "all 0.3s ease",
              boxShadow: open 
                ? "0 4px 12px rgba(0, 0, 0, 0.3), 0 0 0 3px rgba(var(--ifm-color-primary-rgb), 0.2)"
                : "0 2px 8px rgba(0, 0, 0, 0.2)",
              transform: open ? "scale(1.05)" : "scale(1)",
            }}
            onMouseEnter={(e) => {
              if (!open) {
                e.currentTarget.style.transform = "scale(1.1)";
                e.currentTarget.style.boxShadow = "0 4px 12px rgba(0, 0, 0, 0.3)";
              }
            }}
            onMouseLeave={(e) => {
              if (!open) {
                e.currentTarget.style.transform = "scale(1)";
                e.currentTarget.style.boxShadow = "0 2px 8px rgba(0, 0, 0, 0.2)";
              }
            }}
            aria-label="User menu"
          >
            {initial.toUpperCase()}
          </button>
          {open && (
            <>
              {/* Backdrop */}
              <div
                style={{
                  position: "fixed",
                  top: 0,
                  left: 0,
                  right: 0,
                  bottom: 0,
                  zIndex: 9997,
                }}
                onClick={() => setOpen(false)}
              />
              {/* Dropdown Menu */}
              <div
                style={{
                  position: "absolute",
                  top: "3.5rem",
                  right: 0,
                  minWidth: "280px",
                  background:
                    "linear-gradient(145deg, rgba(5, 10, 22, 0.98), rgba(5, 24, 36, 0.98))",
                  borderRadius: "18px",
                  boxShadow:
                    "0 18px 45px rgba(0, 0, 0, 0.85), 0 0 0 1px rgba(0, 212, 255, 0.18)",
                  border: "1px solid rgba(12, 148, 196, 0.45)",
                  padding: "1.4rem 1.35rem 1.25rem",
                  zIndex: 9998,
                  animation: "slideDown 0.2s ease-out",
                  overflow: "hidden",
                  backdropFilter: "blur(16px)",
                }}
              >
                {/* subtle top accent bar */}
                <div
                  style={{
                    position: "absolute",
                    top: 0,
                    left: 0,
                    right: 0,
                    height: "3px",
                    background:
                      "linear-gradient(90deg, rgba(0, 212, 255, 0.1), rgba(0, 255, 157, 0.5), rgba(0, 212, 255, 0.1))",
                  }}
                />
                {/* User Info Section */}
                  <div
                    style={{
                      marginBottom: "1.1rem",
                      paddingBottom: "1rem",
                      borderBottom: "1px solid rgba(148, 163, 184, 0.3)",
                    }}
                  >
                  <div
                    style={{
                      fontSize: "0.75rem",
                      textTransform: "uppercase",
                      letterSpacing: "0.5px",
                      color: "rgba(148, 163, 184, 0.9)",
                      marginBottom: "0.75rem",
                      fontWeight: 600,
                    }}
                  >
                    Account
                  </div>
                  <div
                    style={{
                      display: "flex",
                      alignItems: "center",
                      gap: "0.75rem",
                      marginBottom: "0.5rem",
                    }}
                  >
                    {/* Avatar with glowing ring */}
                    <div
                      style={{
                        position: "relative",
                        width: "3.1rem",
                        height: "3.1rem",
                        flexShrink: 0,
                      }}
                    >
                      <div
                        style={{
                          position: "absolute",
                          inset: 0,
                          borderRadius: "999px",
                          background:
                            "conic-gradient(from 140deg, #00f5ff, #00ff9d, #00b3ff, #00f5ff)",
                          opacity: 0.9,
                          filter: "blur(4px)",
                        }}
                      />
                      <div
                        style={{
                          position: "relative",
                          width: "100%",
                          height: "100%",
                          borderRadius: "999px",
                          padding: "2px",
                          background:
                            "radial-gradient(circle at 30% 0%, rgba(255,255,255,0.22), transparent 55%), linear-gradient(145deg, #020617, #020617)",
                          display: "flex",
                          alignItems: "center",
                          justifyContent: "center",
                          boxShadow:
                            "0 0 0 1px rgba(15, 23, 42, 0.9), 0 10px 25px rgba(0,0,0,0.9)",
                        }}
                      >
                        <div
                          style={{
                            width: "100%",
                            height: "100%",
                            borderRadius: "999px",
                            background:
                              "radial-gradient(circle at 30% 20%, #38bdf8, #0f172a 70%)",
                            display: "flex",
                            alignItems: "center",
                            justifyContent: "center",
                            color: "#e5f4ff",
                            fontWeight: 700,
                            fontSize: "1.25rem",
                          }}
                        >
                          {initial.toUpperCase()}
                        </div>
                      </div>
                    </div>
                    <div style={{ flex: 1, minWidth: 0 }}>
                      <div
                        style={{
                          fontSize: "1rem",
                          fontWeight: 600,
                          color: "#e5e7eb",
                          marginBottom: "0.25rem",
                          overflow: "hidden",
                          textOverflow: "ellipsis",
                          whiteSpace: "nowrap",
                        }}
                      >
                        {session?.user?.name ||
                          session?.session?.user?.name ||
                          session?.user?.email?.split("@")[0] ||
                          session?.session?.user?.email?.split("@")[0] ||
                          "Guest"}
                      </div>
                      {(session?.user?.email || session?.session?.user?.email) && (
                        <div
                          style={{
                            fontSize: "0.875rem",
                            color: "rgba(148, 163, 184, 0.95)",
                            overflow: "hidden",
                            textOverflow: "ellipsis",
                            whiteSpace: "nowrap",
                          }}
                        >
                          {session?.user?.email || session?.session?.user?.email}
                        </div>
                      )}
                    </div>
                  </div>
                </div>

                {/* Menu Items */}
                <div style={{ display: "flex", flexDirection: "column", gap: "0.75rem" }}>
                  <button
                    type="button"
                    onClick={handleSignOut}
                    disabled={signingOut}
                    style={{
                      width: "100%",
                      padding: "0.75rem 1rem",
                      background: signingOut 
                        ? "var(--ifm-color-emphasis-200)" 
                        : "linear-gradient(135deg, #dc2626, #b91c1c)",
                      border: "none",
                      borderRadius: "8px",
                      color: "#ffffff",
                      fontSize: "0.875rem",
                      fontWeight: 600,
                      cursor: signingOut ? "not-allowed" : "pointer",
                      transition: "all 0.2s ease",
                      opacity: signingOut ? 0.7 : 1,
                      display: "flex",
                      alignItems: "center",
                      justifyContent: "center",
                      gap: "0.5rem",
                    }}
                    onMouseEnter={(e) => {
                      if (!signingOut) {
                        e.currentTarget.style.transform = "translateY(-1px)";
                        e.currentTarget.style.boxShadow = "0 4px 12px rgba(220, 38, 38, 0.4)";
                      }
                    }}
                    onMouseLeave={(e) => {
                      if (!signingOut) {
                        e.currentTarget.style.transform = "translateY(0)";
                        e.currentTarget.style.boxShadow = "none";
                      }
                    }}
                  >
                    {signingOut ? (
                      <>
                        <span style={{ display: "inline-block", animation: "spin 1s linear infinite" }}>⟳</span>
                        <span>Signing out...</span>
                      </>
                    ) : (
                      <>
                        <span>🚪</span>
                        <span>Sign Out</span>
                      </>
                    )}
                  </button>
                </div>
              </div>
              <style>{`
                @keyframes slideDown {
                  from {
                    opacity: 0;
                    transform: translateY(-10px);
                  }
                  to {
                    opacity: 1;
                    transform: translateY(0);
                  }
                }
                @keyframes spin {
                  from { transform: rotate(0deg); }
                  to { transform: rotate(360deg); }
                }
              `}</style>
            </>
          )}
        </div>
      ) : (
        <div
          style={{
            display: "flex",
            alignItems: "center",
            gap: "0.5rem",
          }}
        >
          <button
            type="button"
            onClick={() => {
              setAuthModalMode("signin");
              setAuthModalOpen(true);
            }}
            style={{
              padding: "0.5rem 1.25rem",
              background: "transparent",
              border: "1px solid rgba(0, 212, 255, 0.5)",
              borderRadius: "8px",
              color: "#ffffff",
              fontSize: "0.875rem",
              fontWeight: 600,
              cursor: "pointer",
              transition: "all 0.3s ease",
              boxShadow: "0 0 10px rgba(0, 212, 255, 0.3)",
              textShadow: "0 0 10px rgba(0, 212, 255, 0.5)",
              textTransform: "uppercase",
              letterSpacing: "0.05em",
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.borderColor = "#00d4ff";
              e.currentTarget.style.boxShadow = "0 0 20px rgba(0, 212, 255, 0.6)";
              e.currentTarget.style.backgroundColor = "rgba(0, 212, 255, 0.1)";
              e.currentTarget.style.textShadow = "0 0 15px rgba(0, 212, 255, 0.8)";
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.borderColor = "rgba(0, 212, 255, 0.5)";
              e.currentTarget.style.boxShadow = "0 0 10px rgba(0, 212, 255, 0.3)";
              e.currentTarget.style.backgroundColor = "transparent";
              e.currentTarget.style.textShadow = "0 0 10px rgba(0, 212, 255, 0.5)";
            }}
          >
            Login
          </button>
          <AuthModal
            isOpen={authModalOpen}
            onClose={() => setAuthModalOpen(false)}
            initialMode={authModalMode}
            onSuccess={() => {
              showToast(
                authModalMode === "signup"
                  ? "Successfully signed up! Welcome!"
                  : "Successfully signed in! Welcome back!",
                "success",
                3000
              );
            }}
            onError={(error) => {
              showToast(error, "error", 5000);
            }}
          />
        </div>
      )}
    </div>
  );
}


