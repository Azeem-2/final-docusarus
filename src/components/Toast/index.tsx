import React, { useState, useEffect, useCallback } from "react";

export interface Toast {
  id: string;
  message: string;
  type: "success" | "error" | "info";
  duration?: number;
}

interface ToastContextType {
  showToast: (message: string, type?: "success" | "error" | "info", duration?: number) => void;
}

export const ToastContext = React.createContext<ToastContextType>({
  showToast: () => {},
});

export function ToastProvider({ children }: { children: React.ReactNode }) {
  const [toasts, setToasts] = useState<Toast[]>([]);

  const showToast = useCallback(
    (message: string, type: "success" | "error" | "info" = "info", duration: number = 3000) => {
      const id = Math.random().toString(36).substring(7);
      const toast: Toast = { id, message, type, duration };

      setToasts((prev) => [...prev, toast]);

      if (duration > 0) {
        setTimeout(() => {
          removeToast(id);
        }, duration);
      }
    },
    []
  );

  const removeToast = useCallback((id: string) => {
    setToasts((prev) => prev.filter((toast) => toast.id !== id));
  }, []);

  return (
    <ToastContext.Provider value={{ showToast }}>
      {children}
      <ToastContainer toasts={toasts} onRemove={removeToast} />
    </ToastContext.Provider>
  );
}

export function useToast() {
  return React.useContext(ToastContext);
}

interface ToastContainerProps {
  toasts: Toast[];
  onRemove: (id: string) => void;
}

function ToastContainer({ toasts, onRemove }: ToastContainerProps) {
  if (toasts.length === 0) return null;
  
  return (
    <div
      style={{
        position: "fixed",
        top: 0,
        left: 0,
        right: 0,
        bottom: 0,
        zIndex: 10001,
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        flexDirection: "column",
        gap: "0.75rem",
        pointerEvents: "none",
      }}
    >
      {toasts.map((toast) => (
        <ToastItem key={toast.id} toast={toast} onRemove={onRemove} />
      ))}
    </div>
  );
}

interface ToastItemProps {
  toast: Toast;
  onRemove: (id: string) => void;
}

function ToastItem({ toast, onRemove }: ToastItemProps) {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    // Trigger animation
    setTimeout(() => setIsVisible(true), 10);
  }, []);

  const handleClose = () => {
    setIsVisible(false);
    setTimeout(() => onRemove(toast.id), 300);
  };

  const getToastStyles = () => {
    const base = {
      padding: "1rem 1.25rem",
      borderRadius: "8px",
      boxShadow: "var(--ifm-global-shadow-md)",
      display: "flex",
      alignItems: "center",
      justifyContent: "space-between",
      gap: "1rem",
      minWidth: "300px",
      maxWidth: "400px",
      pointerEvents: "auto" as const,
      transition: "all 0.3s ease",
      transform: isVisible ? "translateY(0)" : "translateY(-20px)",
      opacity: isVisible ? 1 : 0,
      background: "var(--ifm-background-surface-color)",
      border: "1px solid var(--ifm-color-emphasis-300)",
    } as const;

    switch (toast.type) {
      case "success":
        return {
          ...base,
          background: "var(--ifm-color-success)",
          color: "var(--ifm-color-success-contrast-background)",
          border: "none",
        };
      case "error":
        return {
          ...base,
          background: "var(--ifm-color-danger)",
          color: "var(--ifm-color-danger-contrast-background)",
          border: "none",
        };
      default:
        return base;
    }
  };

  return (
    <div style={getToastStyles()}>
      <div style={{ flex: 1 }}>
        {toast.type === "success" && "✓ "}
        {toast.type === "error" && "✕ "}
        {toast.message}
      </div>
      <button
        type="button"
        onClick={handleClose}
        style={{
          background: "none",
          border: "none",
          cursor: "pointer",
          fontSize: "1.25rem",
          lineHeight: 1,
          padding: 0,
          width: "1.5rem",
          height: "1.5rem",
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
          opacity: 0.7,
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.opacity = "1";
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.opacity = "0.7";
        }}
      >
        ×
      </button>
    </div>
  );
}

