import React, { useEffect, useRef, useState } from 'react';
import styles from './CardBeamAnimation.module.css';

interface CardBeamAnimationProps {
  children: React.ReactNode;
  className?: string;
}

const codeChars = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789(){}[]<>;:,._-+=!@#$%^&*|\\/\"'`~?";

function generateCode(width: number, height: number): string {
  const randInt = (min: number, max: number) =>
    Math.floor(Math.random() * (max - min + 1)) + min;
  const pick = (arr: string[]) => arr[randInt(0, arr.length - 1)];

  const header = [
    "// compiled preview • scanner demo",
    "/* generated for visual effect – not executed */",
    "const SCAN_WIDTH = 8;",
    "const FADE_ZONE = 35;",
    "const MAX_PARTICLES = 2500;",
    "const TRANSITION = 0.05;",
  ];

  const helpers = [
    "function clamp(n, a, b) { return Math.max(a, Math.min(b, n)); }",
    "function lerp(a, b, t) { return a + (b - a) * t; }",
    "const now = () => performance.now();",
    "function rng(min, max) { return Math.random() * (max - min) + min; }",
  ];

  const library = [...header, ...helpers];
  
  for (let i = 0; i < 40; i++) {
    const n1 = randInt(1, 9);
    const n2 = randInt(10, 99);
    library.push(`const v${i} = (${n1} + ${n2}) * 0.${randInt(1, 9)};`);
  }

  let flow = library.join(" ");
  flow = flow.replace(/\s+/g, " ").trim();
  const totalChars = width * height;
  while (flow.length < totalChars + width) {
    const extra = pick(library).replace(/\s+/g, " ").trim();
    flow += " " + extra;
  }

  let out = "";
  let offset = 0;
  for (let row = 0; row < height; row++) {
    let line = flow.slice(offset, offset + width);
    if (line.length < width) line = line + " ".repeat(width - line.length);
    out += line + (row < height - 1 ? "\n" : "");
    offset += width;
  }
  return out;
}

export default function CardBeamAnimation({ children, className }: CardBeamAnimationProps) {
  const cardRef = useRef<HTMLDivElement>(null);
  const asciiRef = useRef<HTMLDivElement>(null);
  const normalRef = useRef<HTMLDivElement>(null);
  const [code, setCode] = useState<string>('');

  useEffect(() => {
    if (!cardRef.current) return;

    const cardWidth = 400;
    const cardHeight = 250;
    const fontSize = 11;
    const lineHeight = 13;
    const charWidth = 6;
    const width = Math.floor(cardWidth / charWidth);
    const height = Math.floor(cardHeight / lineHeight);
    
    setCode(generateCode(width, height));
  }, []);

  useEffect(() => {
    if (!cardRef.current || !normalRef.current || !asciiRef.current) return;

    let animationFrameId: number;

    const updateClipping = () => {
      const rect = cardRef.current?.getBoundingClientRect();
      if (!rect) return;

      // Use mouse position or center of screen
      let scannerX = window.innerWidth / 2;
      
      // Try to get mouse position if available
      const mouseX = (window as any).lastMouseX;
      if (mouseX !== undefined) {
        scannerX = mouseX;
      }

      const scannerWidth = 8;
      const scannerLeft = scannerX - scannerWidth / 2;
      const scannerRight = scannerX + scannerWidth / 2;

      const cardLeft = rect.left;
      const cardRight = rect.right;
      const cardWidth = rect.width;

      if (cardLeft < scannerRight && cardRight > scannerLeft) {
        const scannerIntersectLeft = Math.max(scannerLeft - cardLeft, 0);
        const scannerIntersectRight = Math.min(scannerRight - cardLeft, cardWidth);

        const normalClipRight = (scannerIntersectLeft / cardWidth) * 100;
        const asciiClipLeft = (scannerIntersectRight / cardWidth) * 100;

        if (normalRef.current) {
          normalRef.current.style.setProperty('--clip-right', `${normalClipRight}%`);
        }
        if (asciiRef.current) {
          asciiRef.current.style.setProperty('--clip-left', `${asciiClipLeft}%`);
        }
      } else {
        // Reset when scanner is not over card
        if (normalRef.current) {
          normalRef.current.style.setProperty('--clip-right', '0%');
        }
        if (asciiRef.current) {
          asciiRef.current.style.setProperty('--clip-left', '100%');
        }
      }

      animationFrameId = requestAnimationFrame(updateClipping);
    };

    const handleScroll = () => {
      updateClipping();
    };

    const handleMouseMove = (e: MouseEvent) => {
      (window as any).lastMouseX = e.clientX;
      updateClipping();
    };

    const handleMouseEnter = () => {
      updateClipping();
    };

    window.addEventListener('scroll', handleScroll, { passive: true });
    window.addEventListener('mousemove', handleMouseMove, { passive: true });
    cardRef.current.addEventListener('mouseenter', handleMouseEnter);
    
    updateClipping();

    return () => {
      window.removeEventListener('scroll', handleScroll);
      window.removeEventListener('mousemove', handleMouseMove);
      cardRef.current?.removeEventListener('mouseenter', handleMouseEnter);
      if (animationFrameId) {
        cancelAnimationFrame(animationFrameId);
      }
    };
  }, [code]);

  return (
    <div ref={cardRef} className={`${styles.cardWrapper} ${className || ''}`}>
      <div ref={normalRef} className={styles.cardNormal}>
        {children}
      </div>
      <div ref={asciiRef} className={styles.cardAscii}>
        <div className={styles.asciiContent}>{code}</div>
      </div>
    </div>
  );
}

