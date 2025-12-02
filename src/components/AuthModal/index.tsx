import React, { useState, useEffect } from "react";
import { authClient } from "@site/src/lib/auth-client";
import clsx from "clsx";

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  initialMode?: "signin" | "signup";
  onSuccess?: () => void;
  onError?: (error: string) => void;
}

export default function AuthModal({
  isOpen,
  onClose,
  initialMode = "signin",
  onSuccess,
  onError,
}: AuthModalProps) {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [name, setName] = useState("");
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);
  const [isSignUp, setIsSignUp] = useState(initialMode === "signup");

  // Reset form when modal opens/closes or mode changes
  useEffect(() => {
    if (isOpen) {
      setIsSignUp(initialMode === "signup");
      setEmail("");
      setPassword("");
      setName("");
      setError("");
    }
  }, [isOpen, initialMode]);

  // Close on Escape key
  useEffect(() => {
    if (!isOpen) return;

    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === "Escape") {
        onClose();
      }
    };

    document.addEventListener("keydown", handleEscape);
    return () => document.removeEventListener("keydown", handleEscape);
  }, [isOpen, onClose]);

  // Prevent body scroll when modal is open
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = "hidden";
    } else {
      document.body.style.overflow = "";
    }
    return () => {
      document.body.style.overflow = "";
    };
  }, [isOpen]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setLoading(true);

    try {
      if (isSignUp) {
        const { data, error } = await authClient.signUp.email({
          email: email.trim(),
          password,
          name: name.trim(),
        });

        if (error) {
          let message = "Failed to sign up";
          if (error.status === 422) {
            message =
              error.message || "Validation error. Please check your input.";
          } else if (error.status === 0 || error.message?.includes("fetch")) {
            message = "Cannot connect to authentication server.";
          } else if (error.message) {
            message = error.message;
          }
          setError(message);
          onError?.(message);
        } else if (data) {
          onSuccess?.();
          onClose();
          // Reload to update auth state
          setTimeout(() => window.location.reload(), 500);
        }
      } else {
        const { data, error } = await authClient.signIn.email({
          email: email.trim(),
          password,
        });

        if (error) {
          let message = "Failed to sign in";
          if (error.status === 422) {
            message = error.message || "Invalid credentials.";
          } else if (error.status === 0 || error.message?.includes("fetch")) {
            message = "Cannot connect to authentication server.";
          } else if (error.message) {
            message = error.message;
          }
          setError(message);
          onError?.(message);
        } else if (data) {
          // Debug: Log the response data
          console.log("[AuthModal] Sign-in response data:", data);
          
          // If data contains a token or session, store it manually as fallback
          if (data.token) {
            localStorage.setItem("better-auth-token", data.token);
          }
          if (data.session) {
            localStorage.setItem("better-auth-session", JSON.stringify(data.session));
          }
          
          onSuccess?.();
          onClose();
          // Reload to update auth state
          setTimeout(() => window.location.reload(), 500);
        }
      }
    } catch (err: any) {
      const errorMessage = err?.message || "An unexpected error occurred.";
      setError(errorMessage);
      onError?.(errorMessage);
    } finally {
      setLoading(false);
    }
  };

  if (!isOpen) return null;

  return (
    <>
      {/* Backdrop with network pattern */}
      <div
        style={{
          position: "fixed",
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          width: "100vw",
          height: "100vh",
          backgroundColor: "rgba(0, 0, 0, 0.85)",
          zIndex: 9998,
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
          padding: "1rem",
          boxSizing: "border-box",
          overflow: "auto",
          backgroundImage: `
            radial-gradient(circle at 20% 30%, rgba(0, 212, 255, 0.1) 0%, transparent 50%),
            radial-gradient(circle at 80% 70%, rgba(0, 255, 255, 0.1) 0%, transparent 50%),
            linear-gradient(135deg, rgba(0, 0, 0, 0.9) 0%, rgba(0, 20, 30, 0.9) 100%)
          `,
        }}
        onClick={onClose}
      >
        {/* Modal */}
        <div
          style={{
            background: "rgba(0, 0, 0, 0.95)",
            borderRadius: "24px",
            border: "1px solid rgba(0, 212, 255, 0.5)",
            boxShadow: `
              0 0 20px rgba(0, 212, 255, 0.3),
              0 0 40px rgba(0, 255, 255, 0.2),
              inset 0 0 20px rgba(0, 212, 255, 0.1)
            `,
            width: "100%",
            maxWidth: "420px",
            maxHeight: "90vh",
            overflowY: "auto",
            position: "relative",
            zIndex: 9999,
            padding: "2rem",
            boxSizing: "border-box",
            margin: "auto",
            flexShrink: 0,
          }}
          onClick={(e) => e.stopPropagation()}
        >
          {/* Header */}
          <div
            style={{
              display: "flex",
              alignItems: "center",
              justifyContent: "space-between",
              marginBottom: "2rem",
            }}
          >
            <h2
              style={{
                margin: 0,
                fontSize: "2rem",
                fontWeight: 700,
                color: "#00d4ff",
                textShadow: "0 0 10px rgba(0, 212, 255, 0.8), 0 0 20px rgba(0, 212, 255, 0.5)",
                letterSpacing: "0.1em",
                textTransform: "uppercase",
              }}
            >
              {isSignUp ? "SIGN UP" : "LOGIN"}
            </h2>
            <button
              type="button"
              onClick={onClose}
              style={{
                background: "transparent",
                border: "1px solid rgba(0, 212, 255, 0.5)",
                fontSize: "1.5rem",
                cursor: "pointer",
                padding: "0.25rem 0.5rem",
                display: "flex",
                alignItems: "center",
                justifyContent: "center",
                color: "#00d4ff",
                borderRadius: "8px",
                width: "2rem",
                height: "2rem",
                transition: "all 0.3s ease",
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.backgroundColor = "rgba(0, 212, 255, 0.2)";
                e.currentTarget.style.boxShadow = "0 0 10px rgba(0, 212, 255, 0.5)";
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.backgroundColor = "transparent";
                e.currentTarget.style.boxShadow = "none";
              }}
            >
              ×
            </button>
          </div>

          {/* Content */}
          <div>
            {error && (
              <div
                style={{
                  marginBottom: "1rem",
                  padding: "0.75rem",
                  background: "rgba(255, 0, 0, 0.1)",
                  border: "1px solid rgba(255, 0, 0, 0.5)",
                  borderRadius: "8px",
                  color: "#ff6b6b",
                  fontSize: "0.9rem",
                  boxShadow: "0 0 10px rgba(255, 0, 0, 0.3)",
                }}
              >
                {error}
              </div>
            )}

            <form onSubmit={handleSubmit}>
              {isSignUp && (
                <div style={{ marginBottom: "1.5rem" }}>
                  <label
                    htmlFor="auth-name"
                    style={{
                      display: "block",
                      marginBottom: "0.5rem",
                      fontWeight: 500,
                      fontSize: "0.9rem",
                      color: "#ffffff",
                    }}
                  >
                    Name
                  </label>
                  <input
                    id="auth-name"
                    type="text"
                    value={name}
                    onChange={(e) => setName(e.target.value)}
                    required
                    disabled={loading}
                    style={{
                      width: "100%",
                      padding: "0.75rem 0",
                      background: "transparent",
                      border: "none",
                      borderBottom: "1px solid rgba(0, 212, 255, 0.5)",
                      borderRadius: "0",
                      fontSize: "0.95rem",
                      color: "#ffffff",
                      outline: "none",
                      transition: "all 0.3s ease",
                    }}
                    onFocus={(e) => {
                      e.currentTarget.style.borderBottomColor = "#00d4ff";
                      e.currentTarget.style.boxShadow = "0 1px 0 0 rgba(0, 212, 255, 0.8)";
                    }}
                    onBlur={(e) => {
                      e.currentTarget.style.borderBottomColor = "rgba(0, 212, 255, 0.5)";
                      e.currentTarget.style.boxShadow = "none";
                    }}
                    placeholder="John Doe"
                  />
                </div>
              )}

              <div style={{ marginBottom: "1.5rem" }}>
                <label
                  htmlFor="auth-email"
                  style={{
                    display: "block",
                    marginBottom: "0.5rem",
                    fontWeight: 500,
                    fontSize: "0.9rem",
                    color: "#ffffff",
                  }}
                >
                  Username
                </label>
                <input
                  id="auth-email"
                  type="email"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                  disabled={loading}
                  style={{
                    width: "100%",
                    padding: "0.75rem 0",
                    background: "transparent",
                    border: "none",
                    borderBottom: "1px solid rgba(0, 212, 255, 0.5)",
                    borderRadius: "0",
                    fontSize: "0.95rem",
                    color: "#ffffff",
                    outline: "none",
                    transition: "all 0.3s ease",
                  }}
                  onFocus={(e) => {
                    e.currentTarget.style.borderBottomColor = "#00d4ff";
                    e.currentTarget.style.boxShadow = "0 1px 0 0 rgba(0, 212, 255, 0.8)";
                  }}
                  onBlur={(e) => {
                    e.currentTarget.style.borderBottomColor = "rgba(0, 212, 255, 0.5)";
                    e.currentTarget.style.boxShadow = "none";
                  }}
                  placeholder="Enter your email"
                />
              </div>

              <div style={{ marginBottom: "1.5rem" }}>
                <div
                  style={{
                    display: "flex",
                    justifyContent: "space-between",
                    alignItems: "center",
                    marginBottom: "0.5rem",
                  }}
                >
                  <label
                    htmlFor="auth-password"
                    style={{
                    fontWeight: 500,
                    fontSize: "0.9rem",
                      color: "#ffffff",
                  }}
                >
                  Password
                </label>
                  <button
                    type="button"
                    onClick={() => {}}
                    style={{
                      background: "none",
                      border: "none",
                      color: "#ffffff",
                      fontSize: "0.85rem",
                      cursor: "pointer",
                      textDecoration: "none",
                    }}
                    onMouseEnter={(e) => {
                      e.currentTarget.style.color = "#00d4ff";
                    }}
                    onMouseLeave={(e) => {
                      e.currentTarget.style.color = "#ffffff";
                    }}
                  >
                    Forgot Password?
                  </button>
                </div>
                <input
                  id="auth-password"
                  type="password"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                  disabled={loading}
                  minLength={8}
                  style={{
                    width: "100%",
                    padding: "0.75rem 0",
                    background: "transparent",
                    border: "none",
                    borderBottom: "1px solid rgba(0, 212, 255, 0.5)",
                    borderRadius: "0",
                    fontSize: "0.95rem",
                    color: "#ffffff",
                    outline: "none",
                    transition: "all 0.3s ease",
                  }}
                  onFocus={(e) => {
                    e.currentTarget.style.borderBottomColor = "#00d4ff";
                    e.currentTarget.style.boxShadow = "0 1px 0 0 rgba(0, 212, 255, 0.8)";
                  }}
                  onBlur={(e) => {
                    e.currentTarget.style.borderBottomColor = "rgba(0, 212, 255, 0.5)";
                    e.currentTarget.style.boxShadow = "none";
                  }}
                  placeholder="Enter your password"
                />
                {isSignUp && (
                  <small
                    style={{
                      display: "block",
                      marginTop: "0.25rem",
                      color: "rgba(255, 255, 255, 0.6)",
                      fontSize: "0.8rem",
                    }}
                  >
                    Must be at least 8 characters
                  </small>
                )}
              </div>

              <div
                style={{
                  display: "flex",
                  alignItems: "center",
                  marginBottom: "2rem",
                }}
              >
                <input
                  type="checkbox"
                  id="remember-me"
                  style={{
                    marginRight: "0.5rem",
                    width: "16px",
                    height: "16px",
                    cursor: "pointer",
                  }}
                />
                <label
                  htmlFor="remember-me"
                  style={{
                    color: "#ffffff",
                    fontSize: "0.9rem",
                    cursor: "pointer",
                  }}
                >
                  Remember me
                </label>
              </div>

              <button
                type="submit"
                disabled={loading}
                style={{
                  width: "100%",
                  padding: "1rem",
                  background: "transparent",
                  border: "1px solid rgba(0, 212, 255, 0.5)",
                  borderRadius: "12px",
                  color: "#ffffff",
                  fontSize: "1rem",
                  fontWeight: 600,
                  textTransform: "uppercase",
                  letterSpacing: "0.1em",
                  cursor: loading ? "not-allowed" : "pointer",
                  marginBottom: "1.5rem",
                  transition: "all 0.3s ease",
                  boxShadow: "0 0 10px rgba(0, 212, 255, 0.3)",
                  textShadow: "0 0 10px rgba(0, 212, 255, 0.5)",
                }}
                onMouseEnter={(e) => {
                  if (!loading) {
                    e.currentTarget.style.borderColor = "#00d4ff";
                    e.currentTarget.style.boxShadow = "0 0 20px rgba(0, 212, 255, 0.6)";
                    e.currentTarget.style.backgroundColor = "rgba(0, 212, 255, 0.1)";
                  }
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.borderColor = "rgba(0, 212, 255, 0.5)";
                  e.currentTarget.style.boxShadow = "0 0 10px rgba(0, 212, 255, 0.3)";
                  e.currentTarget.style.backgroundColor = "transparent";
                }}
              >
                {loading
                  ? "Loading..."
                  : isSignUp
                  ? "SIGN UP"
                  : "SIGN IN"}
              </button>
            </form>

            <div style={{ textAlign: "center" }}>
              <span style={{ color: "#ffffff", fontSize: "0.9rem" }}>
                {isSignUp
                  ? "Already have an account? "
                  : "Don't have an account? "}
              </span>
              <button
                type="button"
                onClick={() => {
                  setIsSignUp(!isSignUp);
                  setError("");
                }}
                style={{
                  background: "none",
                  border: "none",
                  color: "#00d4ff",
                  fontSize: "0.9rem",
                  cursor: "pointer",
                  textDecoration: "none",
                  textShadow: "0 0 10px rgba(0, 212, 255, 0.5)",
                  transition: "all 0.3s ease",
                }}
                onMouseEnter={(e) => {
                  e.currentTarget.style.textShadow = "0 0 15px rgba(0, 212, 255, 0.8)";
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.textShadow = "0 0 10px rgba(0, 212, 255, 0.5)";
                }}
              >
                {isSignUp ? "Sign in" : "Register"}
              </button>
            </div>
          </div>
        </div>
      </div>
    </>
  );
}


