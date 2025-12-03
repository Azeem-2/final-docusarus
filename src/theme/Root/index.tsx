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

    // initial attempt
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
  // Prevent FOUC by setting theme immediately before React hydrates
  if (typeof window !== 'undefined') {
    const html = document.documentElement;
    // Force dark mode and prevent any theme switching
    html.setAttribute('data-theme', 'dark');
    html.setAttribute('data-theme-choice', 'dark');
    // Remove any existing theme classes that might conflict
    html.classList.remove('light-theme', 'dark-theme');
    html.classList.add('dark-theme');
    
    // Override localStorage to prevent theme changes
    try {
      localStorage.setItem('theme', 'dark');
      localStorage.setItem('docusaurus-theme', 'dark');
    } catch (e) {
      // Ignore localStorage errors
    }
  }

  return (
    <PyodideProvider>
      <AnalyticsTracker>
        <AuthProvider>
          <ToastProvider>
            {children}
            <BackgroundAudio />
            <ChatWidget />
          </ToastProvider>
        </AuthProvider>
      </AnalyticsTracker>
    </PyodideProvider>
  );
}
