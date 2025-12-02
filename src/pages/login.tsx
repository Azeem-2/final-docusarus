import React, { useState } from "react";
import Layout from "@theme/Layout";
import { authClient } from "@site/src/lib/auth-client";
import { useHistory } from "@docusaurus/router";
import clsx from "clsx";

export default function LoginPage() {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);
  const [isSignUp, setIsSignUp] = useState(false);
  const [name, setName] = useState("");
  const history = useHistory();

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
            message =
              "Cannot connect to authentication server. Please ensure Better Auth backend is running and CORS is configured.";
          } else if (error.message) {
            message = error.message;
          }
          setError(message);
        } else if (data) {
          // Wait a moment for cookies to be set, then reload to refresh session
          setTimeout(() => {
            const redirectAfterLogin =
              sessionStorage.getItem("redirectAfterLogin") || "/";
            sessionStorage.removeItem("redirectAfterLogin");
            // Reload page to ensure session is fetched with new cookies
            window.location.href = redirectAfterLogin;
          }, 300);
        } else {
          setError(
            "Sign up completed but no confirmation. Please try signing in."
          );
        }
      } else {
        const { data, error } = await authClient.signIn.email({
          email: email.trim(),
          password,
        });

        if (error) {
          setError(error.message || "Failed to sign in");
        } else if (data) {
          // Debug: Log the response data
          console.log("[LoginPage] Sign-in response data:", data);
          
          // If data contains a token or session, store it manually as fallback
          if (data.token) {
            localStorage.setItem("better-auth-token", data.token);
          }
          if (data.session) {
            localStorage.setItem("better-auth-session", JSON.stringify(data.session));
          }
          
          // Wait a moment for cookies to be set, then reload to refresh session
          setTimeout(() => {
            const redirectAfterLogin =
              sessionStorage.getItem("redirectAfterLogin") || "/";
            sessionStorage.removeItem("redirectAfterLogin");
            // Reload page to ensure session is fetched with new cookies
            window.location.href = redirectAfterLogin;
          }, 300);
        } else {
          setError("Sign in completed but no confirmation.");
        }
      }
    } catch (err: any) {
      let message =
        "An unexpected error occurred. Please check the browser console for details.";

      // Handle network/CORS errors specifically
      if (
        err?.message?.includes("Failed to fetch") ||
        err?.message?.includes("NetworkError") ||
        err?.name === "TypeError"
      ) {
        message =
          "Cannot connect to authentication server. Please ensure:\n1. Better Auth backend is running\n2. No firewall is blocking the connection\n3. Check browser console for CORS errors";
      } else if (err?.message) {
        message = err.message;
      } else if (err?.toString) {
        message = err.toString();
      }

      setError(message);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Login" description="Login to access protected content">
      <div
        style={{
          display: "flex",
          justifyContent: "center",
          alignItems: "center",
          minHeight: "100vh",
          padding: "2rem",
          backgroundImage: `
            radial-gradient(circle at 20% 30%, rgba(0, 212, 255, 0.1) 0%, transparent 50%),
            radial-gradient(circle at 80% 70%, rgba(0, 255, 255, 0.1) 0%, transparent 50%),
            linear-gradient(135deg, rgba(0, 0, 0, 0.95) 0%, rgba(0, 20, 30, 0.95) 100%)
          `,
          backgroundColor: "#000000",
        }}
      >
        <div
          style={{
            maxWidth: "420px",
            width: "100%",
            background: "rgba(0, 0, 0, 0.95)",
            padding: "2rem",
            borderRadius: "24px",
            border: "1px solid rgba(0, 212, 255, 0.5)",
            boxShadow: `
              0 0 20px rgba(0, 212, 255, 0.3),
              0 0 40px rgba(0, 255, 255, 0.2),
              inset 0 0 20px rgba(0, 212, 255, 0.1)
            `,
          }}
        >
          <h1
            style={{
              marginBottom: "0.5rem",
              textAlign: "center",
              fontSize: "2rem",
              fontWeight: 700,
              color: "#00d4ff",
              textShadow: "0 0 10px rgba(0, 212, 255, 0.8), 0 0 20px rgba(0, 212, 255, 0.5)",
              letterSpacing: "0.1em",
              textTransform: "uppercase",
            }}
          >
            {isSignUp ? "SIGN UP" : "LOGIN"}
          </h1>
          <p
            style={{
              color: "#ffffff",
              marginBottom: "2rem",
              textAlign: "center",
              fontSize: "0.95rem",
            }}
          >
            Welcome Back
          </p>

          {error && (
            <div
              style={{
                marginBottom: "1rem",
                padding: "0.75rem",
                background: "rgba(255, 0, 0, 0.1)",
                border: "1px solid rgba(255, 0, 0, 0.5)",
                borderRadius: "8px",
                color: "#ff6b6b",
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
                  htmlFor="name"
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
                  id="name"
                  type="text"
                  value={name}
                  onChange={(e) => setName(e.target.value)}
                  required
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
                htmlFor="email"
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
                id="email"
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
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
                  htmlFor="password"
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
                id="password"
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
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
              {loading ? "Loading..." : isSignUp ? "SIGN UP" : "SIGN IN"}
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
    </Layout>
  );
}


