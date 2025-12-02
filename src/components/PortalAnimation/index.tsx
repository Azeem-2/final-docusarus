import React, { useEffect, useRef, useState } from 'react';
import { useHistory } from '@docusaurus/router';
import styles from './PortalAnimation.module.css';

interface PortalAnimationProps {
  targetUrl: string;
  onComplete?: () => void;
}

declare global {
  interface Window {
    THREE: any;
  }
}

export default function PortalAnimation({ targetUrl, onComplete }: PortalAnimationProps) {
  const [showTunnel, setShowTunnel] = useState(false);
  const [threeLoaded, setThreeLoaded] = useState(false);
  const portalCardRef = useRef<HTMLDivElement>(null);
  const tunnelCanvasRef = useRef<HTMLCanvasElement>(null);
  const cardBgCanvasRef = useRef<HTMLCanvasElement>(null);
  const animationFrameRef = useRef<number | undefined>(undefined);
  const sceneRef = useRef<any>(null);
  const cameraRef = useRef<any>(null);
  const rendererRef = useRef<any>(null);
  const tunnelPathRef = useRef<any[]>([]);
  const particlesRef = useRef<any[]>([]);
  const cameraPositionRef = useRef(0);
  const history = useHistory();

  // Load Three.js
  useEffect(() => {
    if (window.THREE) {
      setThreeLoaded(true);
      return;
    }

    const existingScript = document.querySelector('script[src*="three.js"]');
    if (existingScript) {
      existingScript.addEventListener('load', () => {
        setThreeLoaded(true);
      });
      return;
    }

    const script = document.createElement('script');
    script.src = 'https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js';
    script.async = true;
    script.onload = () => {
      if (window.THREE) {
        setThreeLoaded(true);
      }
    };
    script.onerror = () => {
      const fallbackScript = document.createElement('script');
      fallbackScript.src = 'https://unpkg.com/three@0.128.0/build/three.min.js';
      fallbackScript.async = true;
      fallbackScript.onload = () => {
        if (window.THREE) {
          setThreeLoaded(true);
        }
      };
      document.head.appendChild(fallbackScript);
    };
    document.head.appendChild(script);
  }, []);

  // Initialize card background particles
  useEffect(() => {
    if (!cardBgCanvasRef.current) return;

    const canvas = cardBgCanvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const resize = () => {
      canvas.width = canvas.parentElement?.offsetWidth || 300;
      canvas.height = canvas.parentElement?.offsetHeight || 350;
    };
    resize();
    window.addEventListener('resize', resize);

    const particles: any[] = [];
    const particleCount = 50;

    for (let i = 0; i < particleCount; i++) {
      particles.push({
        x: Math.random() * canvas.width,
        y: Math.random() * canvas.height,
        radius: Math.random() * 2 + 1,
        vx: Math.random() * 2 - 1,
        vy: Math.random() * 2 - 1,
        color: `rgba(0, ${Math.floor(Math.random() * 50 + 200)}, ${Math.floor(Math.random() * 50 + 230)}, 0.7)`
      });
    }

    const animate = () => {
      if (!ctx) return;
      requestAnimationFrame(animate);
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      for (let i = 0; i < particleCount; i++) {
        const p = particles[i];
        p.x += p.vx;
        p.y += p.vy;
        if (p.x < 0 || p.x > canvas.width) p.vx *= -1;
        if (p.y < 0 || p.y > canvas.height) p.vy *= -1;

        const gradient = ctx.createRadialGradient(p.x, p.y, 0, p.x, p.y, p.radius * 2);
        gradient.addColorStop(0, 'rgba(255,255,255,1)');
        gradient.addColorStop(1, 'rgba(255,255,255,0)');
        ctx.fillStyle = gradient;
        ctx.beginPath();
        ctx.arc(p.x, p.y, p.radius, 0, Math.PI * 2);
        ctx.fill();

        for (let j = i + 1; j < particleCount; j++) {
          const p2 = particles[j];
          const distance = Math.sqrt(Math.pow(p.x - p2.x, 2) + Math.pow(p.y - p2.y, 2));
          if (distance < 100) {
            ctx.beginPath();
            ctx.strokeStyle = `rgba(0, 217, 255, ${0.1 * (1 - distance / 100)})`;
            ctx.lineWidth = 0.5;
            ctx.moveTo(p.x, p.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.stroke();
          }
        }
      }
    };

    animate();

    return () => {
      window.removeEventListener('resize', resize);
    };
  }, []);

  // Create circular path like original
  const createCircularPath = () => {
    const points = [];
    const totalPoints = 200;
    const controlPoints = [
      new window.THREE.Vector3(0, 0, 0),
      new window.THREE.Vector3(20, 10, -50),
      new window.THREE.Vector3(40, -10, -100),
      new window.THREE.Vector3(60, 15, -150),
      new window.THREE.Vector3(50, -5, -200),
      new window.THREE.Vector3(0, 0, -250),
      new window.THREE.Vector3(-100, 0, -200),
      new window.THREE.Vector3(-150, 0, -100),
      new window.THREE.Vector3(-100, 0, 0),
      new window.THREE.Vector3(-50, 10, 100),
      new window.THREE.Vector3(-20, -10, 150),
      new window.THREE.Vector3(0, 0, 200)
    ];
    const curve = new window.THREE.CatmullRomCurve3(controlPoints);
    curve.tension = 0.1;
    for (let i = 0; i < totalPoints; i++) {
      const t = i / (totalPoints - 1);
      points.push(curve.getPoint(t));
    }
    return points;
  };

  // Initialize tunnel animation - EXACTLY like original
  useEffect(() => {
    if (!showTunnel || !tunnelCanvasRef.current) {
      return;
    }

    if (tunnelCanvasRef.current) {
      tunnelCanvasRef.current.style.display = 'block';
    }

    if (!threeLoaded || !window.THREE) {
      const checkThree = setInterval(() => {
        if (window.THREE) {
          setThreeLoaded(true);
          clearInterval(checkThree);
        }
      }, 100);
      setTimeout(() => clearInterval(checkThree), 5000);
      return () => clearInterval(checkThree);
    }

    const canvas = tunnelCanvasRef.current;
    const w = window.innerWidth;
    const h = window.innerHeight;
    
    // EXACT original values
    const cameraSpeed = 0.00015;
    const lightSpeed = 0.001;
    const tubularSegments = 1200;
    const radialSegments = 12;
    const tubeRadius = 3;

    const renderer = new window.THREE.WebGLRenderer({
      canvas,
      antialias: true,
      alpha: true,
      powerPreference: "high-performance"
    });
    renderer.setSize(w, h);
    
    const scene = new window.THREE.Scene();
    scene.fog = new window.THREE.FogExp2(0x000000, 0.005);
    const camera = new window.THREE.PerspectiveCamera(60, w / h, 0.1, 1000);

    sceneRef.current = scene;
    cameraRef.current = camera;
    rendererRef.current = renderer;

    // Create stars field
    const starsCount = 2000;
    const starsPositions = new Float32Array(starsCount * 3);
    for (let i = 0; i < starsCount; i++) {
      starsPositions[i * 3] = window.THREE.MathUtils.randFloatSpread(1500);
      starsPositions[i * 3 + 1] = window.THREE.MathUtils.randFloatSpread(1500);
      starsPositions[i * 3 + 2] = window.THREE.MathUtils.randFloatSpread(1500);
    }
    const starsGeometry = new window.THREE.BufferGeometry();
    starsGeometry.setAttribute('position', new window.THREE.BufferAttribute(starsPositions, 3));
    
    // Create circle texture for stars
    const starCanvas = document.createElement('canvas');
    starCanvas.width = 64;
    starCanvas.height = 64;
    const starCtx = starCanvas.getContext('2d');
    if (starCtx) {
      starCtx.fillStyle = '#ffffff';
      starCtx.beginPath();
      starCtx.arc(32, 32, 2, 0, Math.PI * 2);
      starCtx.fill();
    }
    const starsTexture = new window.THREE.CanvasTexture(starCanvas);
    const starsMaterial = new window.THREE.PointsMaterial({
      color: 0xffffff,
      size: 1,
      map: starsTexture,
      transparent: true
    });
    const starField = new window.THREE.Points(starsGeometry, starsMaterial);
    scene.add(starField);

    // Create tunnel path
    const organicPoints = createCircularPath();
    const path = new window.THREE.CatmullRomCurve3(organicPoints);
    
    const tubeGeometry = new window.THREE.TubeBufferGeometry(
      path,
      tubularSegments,
      tubeRadius,
      radialSegments,
      false
    );

    // Add colors to tunnel vertices - EXACT original colors
    const colors: number[] = [];
    for (let i = 0; i < tubeGeometry.attributes.position.count; i++) {
      const color = new window.THREE.Color(i % 2 === 0 ? '#00d9ff' : '#0066ff');
      colors.push(color.r, color.g, color.b);
    }
    tubeGeometry.setAttribute('color', new window.THREE.Float32BufferAttribute(colors, 3));

    const material = new window.THREE.MeshLambertMaterial({
      side: window.THREE.BackSide,
      vertexColors: true,
      wireframe: true,
      emissive: 0x333333,
      emissiveIntensity: 0.4
    });
    const tube = new window.THREE.Mesh(tubeGeometry, material);
    scene.add(tube);

    // Add lights - EXACT original
    const mainLight = new window.THREE.PointLight(0xffffff, 1, 50);
    scene.add(mainLight);
    scene.add(new window.THREE.AmbientLight(0x555555));

    const lightColors = [0x00d9ff, 0x0066ff, 0x00d9ff, 0x0066ff, 0xffffff];
    const lights: any[] = [];
    for (let i = 0; i < 5; i++) {
      const offset = i * 0.15 + (i % 3) * 0.05;
      const light = new window.THREE.PointLight(lightColors[i], 1.2, 20);
      lights.push(light);
      scene.add(light);
    }

    // Animation variables
    let pct = 0;
    let pct2 = 0;

    // Animation loop - EXACT original
    const render = () => {
      animationFrameRef.current = requestAnimationFrame(render);

      pct += cameraSpeed;
      if (pct > 1) pct = 0;

      pct2 += lightSpeed;
      if (pct2 > 1) pct2 = 0;

      const currentPoint = path.getPoint(pct);
      const nextPoint = path.getPoint(Math.min(pct + 0.01, 1));
      
      camera.position.copy(currentPoint);
      camera.lookAt(nextPoint);

      // Update lights along path
      lights.forEach((light, i) => {
        const lightPct = (pct2 + i * 0.15) % 1;
        const lightPoint = path.getPoint(lightPct);
        light.position.copy(lightPoint);
      });

      renderer.render(scene, camera);
    };

    render();

    const handleResize = () => {
      const newW = window.innerWidth;
      const newH = window.innerHeight;
      camera.aspect = newW / newH;
      camera.updateProjectionMatrix();
      renderer.setSize(newW, newH);
    };

    window.addEventListener('resize', handleResize);

    return () => {
      window.removeEventListener('resize', handleResize);
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
      renderer.dispose();
    };
  }, [showTunnel, threeLoaded]);

  // Prevent body scroll
  useEffect(() => {
    document.body.style.overflow = 'hidden';
    document.documentElement.style.overflow = 'hidden';
    
    return () => {
      document.body.style.overflow = '';
      document.documentElement.style.overflow = '';
    };
  }, []);

  // Start portal animation on mount
  useEffect(() => {
    const startPortal = () => {
      setTimeout(() => {
        setShowTunnel(true);
        
        setTimeout(() => {
          if (tunnelCanvasRef.current) {
            tunnelCanvasRef.current.classList.add(styles.active);
          }
          if (portalCardRef.current) {
            portalCardRef.current.classList.add(styles.zoomIn);
          }
        }, 200);
        
        setTimeout(() => {
          if (portalCardRef.current) {
            portalCardRef.current.style.display = 'none';
          }
          
          setTimeout(() => {
            if (onComplete) {
              onComplete();
            }
            history.push(targetUrl);
          }, 5000);
        }, 2500);
      }, 1000);
    };

    startPortal();
  }, [targetUrl, history, onComplete]);

  return (
    <div className={styles.portalOverlay}>
      <div ref={portalCardRef} className={styles.portalCard} id="portalCard">
        <div className={styles.gooeyEffect}>
          <div className={styles.gooeyBlob}></div>
          <div className={styles.gooeyBlob}></div>
          <div className={styles.gooeyBlob}></div>
          <div className={styles.gooeyBlob}></div>
        </div>
        <div className={styles.portalContent} id="portalContent">
          <h1>ENTER<br/>THE PORTAL</h1>
          <button className={styles.portalButton} id="portalButton">GO</button>
        </div>
        <canvas ref={cardBgCanvasRef} className={styles.cardBg} id="cardBgEffect"></canvas>
      </div>
      
      <div className={`${styles.tunnelContainer} ${showTunnel ? styles.active : ''}`} id="tunnelContainer">
        <canvas 
          ref={tunnelCanvasRef} 
          className={`${styles.tunnelCanvas} ${showTunnel ? styles.active : ''}`} 
          id="tunnelCanvas"
          style={{ display: showTunnel ? 'block' : 'none' }}
        ></canvas>
      </div>
    </div>
  );
}
