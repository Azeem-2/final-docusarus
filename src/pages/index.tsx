import type { ReactNode } from "react";
import { useEffect, useRef, useState } from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import Heading from "@theme/Heading";
import GhostCard from "@site/src/components/GhostCard";
import PortalAnimation from "@site/src/components/PortalAnimation";

import styles from "./index.module.css";

function HeroEyeBackground() {
  const irisRef = useRef<HTMLDivElement | null>(null);
  const pupilRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    const iris = irisRef.current;
    const pupil = pupilRef.current;
    if (!iris || !pupil) return;

    let currentX = 0;
    let currentY = 0;
    let targetX = 0;
    let targetY = 0;
    let animationFrameId: number | null = null;

    const handleMouseMove = (e: MouseEvent) => {
      const centerX = window.innerWidth / 2;
      const centerY = window.innerHeight / 2;
      const rawX = e.clientX - centerX;
      const rawY = e.clientY - centerY;

      const maxDist = 60;
      const angle = Math.atan2(rawY, rawX);
      const dist = Math.min(Math.sqrt(rawX * rawX + rawY * rawY) / 8, maxDist);
      targetX = Math.cos(angle) * dist;
      targetY = Math.sin(angle) * dist;
    };

    const animate = () => {
      currentX += (targetX - currentX) * 0.08;
      currentY += (targetY - currentY) * 0.08;

      iris.style.transform = `translate(-50%, -50%) translate(${currentX}px, ${currentY}px)`;
      pupil.style.transform = `translate(-50%, -50%) translate(${currentX * 0.25}px, ${currentY * 0.25}px) scaleX(0.65)`;

      animationFrameId = window.requestAnimationFrame(animate);
    };

    window.addEventListener("mousemove", handleMouseMove);
    animationFrameId = window.requestAnimationFrame(animate);

    return () => {
      window.removeEventListener("mousemove", handleMouseMove);
      if (animationFrameId !== null) {
        window.cancelAnimationFrame(animationFrameId);
      }
    };
  }, []);

  return (
    <div className={styles.heroEyeScene} aria-hidden="true">
      <div className={styles.heroEyeEyeball}>
        <div ref={irisRef} className={styles.heroEyeIris}>
          <div className={styles.heroEyeIrisTexture} />
          <div className={styles.heroEyeInnerGlow} />
          <div ref={pupilRef} className={styles.heroEyePupil} />
          <div className={styles.heroEyeHighlightPrimary} />
          <div className={styles.heroEyeHighlightSecondary} />
        </div>
      </div>
    </div>
  );
}

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  const [showPortal, setShowPortal] = useState(false);

  const handleStartReading = (e: React.MouseEvent) => {
    e.preventDefault();
    setShowPortal(true);
  };

  return (
    <>
      <header className={styles.heroBanner}>
        <HeroEyeBackground />
        <div className="container">
          <div className={styles.heroContent}>
            <div className={styles.heroTextContent}>
              <div className={styles.heroBadge}>
                <span className={styles.heroBadgeDot}></span>
                <span>Robotics & AI</span>
              </div>
              <Heading as="h1" className={styles.heroTitle}>
                <span className={styles.heroTitleMain}>Robotic Intelligence</span>
                <span className={styles.heroTitleSub}>AI-Driven Development</span>
              </Heading>
              <p className={styles.heroDescription}>
                Build intelligent robots with Python, TypeScript, and AI.
                Master physical AI systems that see, think, and act autonomously.
              </p>

              <div className={styles.heroButtons}>
                <button
                  className={clsx(
                    "button button--primary button--lg",
                    styles.ctaButton
                  )}
                  onClick={handleStartReading}
                >
                  Start Reading
                </button>
              </div>
            </div>
          </div>
        </div>
      </header>
      {showPortal && (
        <PortalAnimation
          targetUrl="/docs/toc"
          onComplete={() => setShowPortal(false)}
        />
      )}
    </>
  );
}

function Feature({
  title,
  description,
  icon,
  featured,
}: {
  title: string;
  description: string;
  icon: string;
  featured?: boolean;
}) {
  return (
    <div className={clsx(styles.feature, featured && styles.featureFeatured)}>
      <div className={styles.featureIcon}>{icon}</div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

function AISpectrumSection() {
  const cards = [
    {
      icon: "🔧",
      title: "Programmed",
      description: "Rule-based robots with fixed behaviors and deterministic responses",
      highlighted: false,
    },
    {
      icon: "🤖",
      title: "AI-Enhanced",
      description: "Robots powered by AI for intelligent control and decision-making",
      highlighted: true,
    },
    {
      icon: "🧠",
      title: "Autonomous",
      description: "Self-learning robots that adapt to situations and environments",
      highlighted: true,
    },
    {
      icon: "👁️",
      title: "Vision Systems",
      description: "Advanced computer vision for object recognition and navigation",
      highlighted: false,
    },
    {
      icon: "🦾",
      title: "Humanoid Robotics",
      description: "Human-like robots with natural movement and interaction capabilities",
      highlighted: true,
    },
    {
      icon: "⚡",
      title: "Real-Time Control",
      description: "Low-latency control systems for responsive robotic actions",
      highlighted: false,
    },
    {
      icon: "🌐",
      title: "ROS 2 Integration",
      description: "Robot Operating System 2 for modular and scalable robotics",
      highlighted: false,
    },
    {
      icon: "🔬",
      title: "Simulation",
      description: "Virtual environments for testing and training before deployment",
      highlighted: false,
    },
  ];

  return (
    <section className={styles.spectrumSection}>
      <div className="container">
        <div className={styles.spectrumHeader}>
          <Heading as="h2" className={styles.spectrumTitle}>
            Robotics Development Spectrum
          </Heading>
        </div>

        <div className={styles.spectrumCards}>
          {cards.map((card, index) => (
            <GhostCard key={index} className={styles.ghostCardWrapper}>
              <div className={clsx(styles.spectrumCard, card.highlighted && styles.spectrumCardHighlight)}>
                <div className={styles.spectrumIcon}>{card.icon}</div>
                <h3 className={styles.spectrumCardTitle}>{card.title}</h3>
                <p className={styles.spectrumCardDescription}>{card.description}</p>
          </div>
            </GhostCard>
          ))}
        </div>

      </div>
    </section>
  );
}

function FeaturesSection() {
  return (
    <section className={styles.features}>
      <div className="container">
        {/* Section Header */}
        <div className={styles.featuresHeader}>
          <Heading as="h2" className={styles.featuresHeading}>
            Core Capabilities
          </Heading>
        </div>

        {/* Features Grid */}
        <div className={styles.featuresGrid}>
          <Feature
            icon="�"
            icon="🤖"
            title="Humanoid Robotics"
            description="Build humanoid robots with AI-powered control and natural interaction."
            featured={true}
          />
          <Feature
            icon="🧠"
            title="AI-Powered Control"
            description="Integrate LLMs for planning and decision-making in robotic systems."
          />
          <Feature
            icon="⚙️"
            title="ROS 2 & Simulation"
            description="Master Robot Operating System 2 and simulate before deployment."
          />
        </div>
      </div>
    </section>
  );
}

function MaturityLevelsSection() {
  return (
    <section className={styles.maturitySection}>
      <div className="container">
        <div className={styles.maturityHeader}>
          <div className={styles.maturityLabel}>Your Robotics Journey</div>
          <Heading as="h2" className={styles.maturityTitle}>
            Robotics Development Maturity
          </Heading>
          <p className={styles.maturitySubtitle}>
            From basic automation to fully autonomous systems. Understand where you are
            and where you're heading in robotic intelligence.
          </p>
        </div>

        <div className={styles.maturityLevels}>
          <div className={clsx(styles.maturityLevel, styles.maturityLevelHighlight)}>
            <div className={styles.maturityLevelNumber}>1</div>
            <h3 className={styles.maturityLevelTitle}>AI-Powered</h3>
            <p className={styles.maturityLevelDescription}>
              Robots with AI for planning and decision-making
            </p>
          </div>

          <div className={clsx(styles.maturityLevel, styles.maturityLevelHighlight)}>
            <div className={styles.maturityLevelNumber}>2</div>
            <h3 className={styles.maturityLevelTitle}>Autonomous</h3>
            <p className={styles.maturityLevelDescription}>
              Self-learning robots that adapt to situations
            </p>
          </div>

          <div className={styles.maturityLevel}>
            <div className={styles.maturityLevelNumber}>3</div>
            <h3 className={styles.maturityLevelTitle}>Embodied Intelligence</h3>
            <p className={styles.maturityLevelDescription}>
              General-purpose robots with advanced capabilities
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

function ParadigmShift() {
  return (
    <section className={styles.paradigmSection}>
      <div className="container">
        <div className={styles.paradigmContent}>
          {/* Section Header */}
          <div className={styles.paradigmHeader}>
            <div className={styles.paradigmLabel}>The Robotics Revolution</div>
            <Heading as="h2" className={styles.paradigmTitle}>
              From Programmed Machines to Intelligent Agents
              <br />
              <span className={styles.paradigmTitleAccent}>
                From Code to Physical Reality
              </span>
            </Heading>
            <p className={styles.paradigmSubtitle}>
              The future of robotics isn't about pre-programmed behaviors—it's about
              building intelligent robots that see, think, and adapt. Learn to create
              physical AI systems that operate autonomously in the real world.
            </p>
          </div>

          {/* Minimal comparison */}
          <div className={styles.comparisonGrid}>
            <div className={styles.comparisonCard}>
              <div className={styles.comparisonIcon}>🔧</div>
              <h3 className={styles.comparisonLabel}>Traditional</h3>
              <p className={styles.comparisonDescription}>
                Pre-programmed behaviors, controlled environments
              </p>
            </div>

            <div className={styles.comparisonDivider}>
              <span>→</span>
            </div>

            <div className={clsx(styles.comparisonCard, styles.comparisonCardHighlight)}>
              <div className={styles.comparisonIcon}>🤖</div>
              <h3 className={styles.comparisonLabel}>AI-Powered</h3>
              <p className={styles.comparisonDescription}>
                Natural language control, adaptive behaviors, real-world deployment
              </p>
            </div>
          </div>

          {/* Minimal CTA */}
          <div className={styles.paradigmCTA}>
            <Link
              className={clsx(
                "button button--primary button--lg",
                styles.paradigmCTAButton
              )}
              to="/docs/preface-agent-native"
            >
              Start Building Robots →
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Robotic Intelligence & AI-Driven Development"
      description="Build intelligent robots with Python, TypeScript, and AI. From humanoid robotics to autonomous systems – master physical AI that operates in the real world."
    >
      <HomepageHeader />
      <FeaturesSection />
      <AISpectrumSection />
    </Layout>
  );
}
