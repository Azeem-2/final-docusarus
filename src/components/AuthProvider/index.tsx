import React, { createContext, useContext, ReactNode } from "react";
import { useSession } from "@site/src/lib/auth-client";

interface AuthContextType {
  session: any;
  isAuthenticated: boolean;
  isLoading: boolean;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function AuthProvider({ children }: { children: ReactNode }) {
  const { data: session, isPending: isLoading } = useSession();

  // Debug logging in development
  if (typeof window !== "undefined" && process.env.NODE_ENV === "development") {
    console.log("[AuthProvider] Session data:", session);
    console.log("[AuthProvider] Is loading:", isLoading);
    
    // Check cookies from the auth domain
    const allCookies = document.cookie;
    console.log("[AuthProvider] All cookies:", allCookies || "(none)");
    
    // Check if cookies are being set (they should be visible if SameSite=None is working)
    if (!allCookies || allCookies.trim() === "") {
      console.warn("[AuthProvider] ⚠️ No cookies found! Backend cookies may not be set properly.");
      console.warn("[AuthProvider] Check Network tab → Response Headers → Set-Cookie");
    }
    
    // Check for manually stored session as fallback
    const storedSession = localStorage.getItem("better-auth-session");
    if (storedSession) {
      console.log("[AuthProvider] Found stored session fallback:", storedSession);
    }
  }

  // Better Auth returns session in different structures, handle both
  let sessionData = session?.session || session || null;
  
  // Fallback: If no session from cookies, check localStorage
  if (!sessionData && typeof window !== "undefined") {
    const storedSession = localStorage.getItem("better-auth-session");
    if (storedSession) {
      try {
        sessionData = JSON.parse(storedSession);
        console.log("[AuthProvider] Using stored session as fallback");
      } catch (e) {
        console.error("[AuthProvider] Failed to parse stored session:", e);
      }
    }
  }
  
  const isAuthenticated = !!sessionData;

  const value: AuthContextType = {
    session: sessionData,
    isAuthenticated,
    isLoading,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error("useAuth must be used within an AuthProvider");
  }
  return context;
}


