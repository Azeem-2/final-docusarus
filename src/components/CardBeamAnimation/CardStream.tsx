import React, { useEffect, useRef } from 'react';
import CardBeamAnimation from './index';
import styles from './CardStream.module.css';

interface Card {
  icon: string;
  title: string;
  description: string;
  highlighted?: boolean;
}

interface CardStreamProps {
  cards: Card[];
  cardClassName?: string;
  cardHighlightClassName?: string;
  iconClassName?: string;
  titleClassName?: string;
  descriptionClassName?: string;
}

export default function CardStream({
  cards,
  cardClassName = '',
  cardHighlightClassName = '',
  iconClassName = '',
  titleClassName = '',
  descriptionClassName = '',
}: CardStreamProps) {
  const cardLineRef = useRef<HTMLDivElement>(null);
  const positionRef = useRef(0);
  const velocityRef = useRef(120);
  const directionRef = useRef(-1);
  const isAnimatingRef = useRef(true);
  const lastTimeRef = useRef(performance.now());
  const animationFrameRef = useRef<number>();

  // Duplicate cards for seamless loop
  const duplicatedCards = [...cards, ...cards, ...cards];

  useEffect(() => {
    if (!cardLineRef.current) return;

    const animate = () => {
      const currentTime = performance.now();
      const deltaTime = (currentTime - lastTimeRef.current) / 1000;
      lastTimeRef.current = currentTime;

      if (isAnimatingRef.current) {
        positionRef.current += velocityRef.current * directionRef.current * deltaTime;
        
        const containerWidth = window.innerWidth;
        const cardWidth = 500; // Card width (450px) + gap (50px)
        const totalWidth = cardWidth * duplicatedCards.length;

        // Loop animation - seamless infinite scroll
        if (positionRef.current < -totalWidth + containerWidth) {
          positionRef.current += totalWidth;
        } else if (positionRef.current > containerWidth) {
          positionRef.current -= totalWidth;
        }

        if (cardLineRef.current) {
          cardLineRef.current.style.transform = `translateX(${positionRef.current}px)`;
        }
      }

      animationFrameRef.current = requestAnimationFrame(animate);
    };

    animate();

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [duplicatedCards.length]);

  return (
    <div className={styles.cardStream}>
      <div ref={cardLineRef} className={styles.cardLine}>
        {duplicatedCards.map((card, index) => (
          <CardBeamAnimation key={`${card.title}-${index}`} className={styles.cardWrapper}>
            <div className={`${cardClassName} ${card.highlighted ? cardHighlightClassName : ''}`}>
              <div className={iconClassName}>{card.icon}</div>
              <h3 className={titleClassName}>{card.title}</h3>
              <p className={descriptionClassName}>{card.description}</p>
            </div>
          </CardBeamAnimation>
        ))}
      </div>
    </div>
  );
}

