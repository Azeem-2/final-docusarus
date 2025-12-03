/**
 * Docusaurus Root Component
 *
 * This component wraps the entire site with:
 * 1. PyodideProvider - Enables direct Pyodide integration for interactive Python execution
 * 2. AnalyticsTracker - Tracks user interactions (page views, scroll depth, etc.)
 *
 * GA4 is configured via the GA4_MEASUREMENT_ID environment variable.
 * If not set, analytics will not load.
 */

import React, { useEffect, useRef } from 'react';
import { PyodideProvider } from '@/contexts/PyodideContext';
import { AnalyticsTracker } from '@/components/AnalyticsTracker';
import { ChatWidget } from '@/components/ChatWidget';
import { AuthProvider } from '@site/src/components/AuthProvider';
import { ToastProvider } from '@site/src/components/Toast';

function BackgroundAudio() {
  const audioRef = useRef<HTMLAudioElement | null>(null);

  useEffect(() => {
    const audio = audioRef.current;
    if (!audio) {
      console.log("[BG-AUDIO] <audio> element not found");
      return;
    }

    // Full volume as requested (user can still control via system/browser)
    audio.volume = 1.0;

    const tryPlay = () => {
      audio
        .play()
        .then(() => {
          console.log("[BG-AUDIO] Playback started");
        })
        .catch((err) => {
          console.log("[BG-AUDIO] Autoplay blocked or error:", err?.message ?? err);
        });
    };

    // Initial attempt
    tryPlay();

    const handleFirstInteraction = () => {
      console.log("[BG-AUDIO] First user interaction, retrying play");
      tryPlay();
      window.removeEventListener('click', handleFirstInteraction);
      window.removeEventListener('keydown', handleFirstInteraction);
    };

    window.addEventListener('click', handleFirstInteraction);
    window.addEventListener('keydown', handleFirstInteraction);

    return () => {
      window.removeEventListener('click', handleFirstInteraction);
      window.removeEventListener('keydown', handleFirstInteraction);
    };
  }, []);

  return (
    <audio
      ref={audioRef}
      src="/sound1.mp3"
      autoPlay
      loop
      playsInline
      onCanPlay={() => console.log("[BG-AUDIO] Can play event fired")}
      onPlay={() => console.log("[BG-AUDIO] onPlay event fired")}
      onError={(e) => console.log("[BG-AUDIO] Audio error event", e)}
    />
  );
}

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <PyodideProvider>
      <AnalyticsTracker>
        <AuthProvider>
          <ToastProvider>
            {children}
          </ToastProvider>
        </AuthProvider>
      </AnalyticsTracker>
      <BackgroundAudio />
      <ChatWidget />
    </PyodideProvider>
  );
}
