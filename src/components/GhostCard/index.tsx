import React, { useEffect, useRef, useState } from 'react';
import styles from './GhostCard.module.css';

interface GhostCardProps {
  children: React.ReactNode;
  className?: string;
}

declare global {
  interface Window {
    THREE: any;
  }
}

export default function GhostCard({ children, className }: GhostCardProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const [threeLoaded, setThreeLoaded] = useState(false);
  const sceneRef = useRef<any>(null);
  const cameraRef = useRef<any>(null);
  const rendererRef = useRef<any>(null);
  const cardMeshRef = useRef<any>(null);
  const animationFrameRef = useRef<number>();

  // Load Three.js
  useEffect(() => {
    if (window.THREE) {
      setThreeLoaded(true);
      return;
    }

    const existingScript = document.querySelector('script[src*="three.js"]');
    if (existingScript) {
      existingScript.addEventListener('load', () => setThreeLoaded(true));
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
    document.head.appendChild(script);
  }, []);

  // Initialize 3D card
  useEffect(() => {
    if (!threeLoaded || !window.THREE || !containerRef.current) return;

    const container = containerRef.current;
    const width = container.offsetWidth;
    const height = container.offsetHeight;

    // Scene setup - EXACT like original
    const scene = new window.THREE.Scene();
    const camera = new window.THREE.PerspectiveCamera(30, width / height, 1, 10000);
    camera.position.z = 100;
    
    const renderer = new window.THREE.WebGLRenderer({ 
      antialias: true, 
      alpha: true,
      powerPreference: "high-performance"
    });
    renderer.setPixelRatio(2);
    renderer.setSize(width, height);
    renderer.setClearColor(0x000000, 0);
    renderer.autoClear = false;
    container.appendChild(renderer.domElement);

    // OrbitControls for camera rotation - EXACT like original
    let controls: any = null;
    let mouseX = 0;
    let mouseY = 0;
    
    // Manual mouse tracking for camera rotation (like OrbitControls)
    const handleMouseMove = (e: MouseEvent) => {
      const rect = container.getBoundingClientRect();
      mouseX = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      mouseY = ((e.clientY - rect.top) / rect.height) * 2 - 1;
    };
    
    const handleMouseDown = (e: MouseEvent) => {
      e.preventDefault();
      const rect = container.getBoundingClientRect();
      const startX = e.clientX;
      const startY = e.clientY;
      const startMouseX = mouseX;
      const startMouseY = mouseY;
      
      const handleMouseDrag = (e: MouseEvent) => {
        const deltaX = (e.clientX - startX) / rect.width;
        const deltaY = (e.clientY - startY) / rect.height;
        mouseX = startMouseX + deltaX * 2;
        mouseY = startMouseY + deltaY * 2;
      };
      
      const handleMouseUp = () => {
        document.removeEventListener('mousemove', handleMouseDrag);
        document.removeEventListener('mouseup', handleMouseUp);
      };
      
      document.addEventListener('mousemove', handleMouseDrag);
      document.addEventListener('mouseup', handleMouseUp);
    };
    
    container.addEventListener('mousemove', handleMouseMove);
    container.addEventListener('mousedown', handleMouseDown);
    
    // Camera rotation based on mouse position
    const updateCameraRotation = () => {
      const spherical = new window.THREE.Spherical();
      spherical.radius = 100;
      spherical.theta = Math.PI / 2 + mouseY * 0.8; // Vertical rotation
      spherical.phi = Math.PI / 2 + mouseX * 0.8; // Horizontal rotation
      
      const newPosition = new window.THREE.Vector3();
      newPosition.setFromSpherical(spherical);
      camera.position.lerp(newPosition, 0.1);
      camera.lookAt(0, 0, 0);
    };
    
    controls = { update: updateCameraRotation };

    sceneRef.current = scene;
    cameraRef.current = camera;
    rendererRef.current = renderer;

    // Shader uniforms
    const uniforms = {
      time: { value: 0.0 },
      resolution: { value: new window.THREE.Vector2(width, height) },
      color0: { value: new window.THREE.Vector3(0, 217, 255).divideScalar(255) }, // Cyan
      color1: { value: new window.THREE.Vector3(0, 102, 255).divideScalar(255) }, // Blue
    };

    // Vertex shader
    const vert = `
      varying vec2 vUv;
      varying vec3 camPos;
      varying vec3 eyeVector;
      varying vec3 vNormal;
      void main() {
        vUv = uv;
        camPos = cameraPosition;
        vNormal = normal;
        vec4 worldPosition = modelViewMatrix * vec4(position, 1.0);
        eyeVector = normalize(worldPosition.xyz - abs(cameraPosition));
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `;

    // Fragment shader - ghost effect (exact like original)
    const frag = `
      uniform float time;
      uniform vec2 resolution;
      uniform vec3 color0;
      uniform vec3 color1;
      varying vec2 vUv;
      varying vec3 camPos;
      varying vec3 eyeVector;
      varying vec3 vNormal;

      float Fresnel(vec3 eyeVector, vec3 worldNormal) {
        return pow(1.0 + dot(eyeVector, worldNormal), 1.80);
      }

      float rand(vec2 n) {
        return fract(sin(dot(n, vec2(12.9898, 4.1414))) * 43758.5453);
      }

      float noise(vec2 p) {
        vec2 ip = floor(p);
        vec2 u = fract(p);
        u = u * u * (3.0 - 2.0 * u);
        float res = mix(
          mix(rand(ip), rand(ip + vec2(1.0, 0.0)), u.x),
          mix(rand(ip + vec2(0.0, 1.0)), rand(ip + vec2(1.0, 1.0)), u.x),
          u.y
        );
        return res * res;
      }

      float fbm(vec2 x) {
        float v = 0.0;
        float a = 0.5;
        vec2 shift = vec2(100.0);
        mat2 rot = mat2(cos(0.5), sin(0.5), -sin(0.5), cos(0.5));
        for (int i = 0; i < 5; ++i) {
          v += a * noise(x);
          x = rot * x * 2.0 + shift;
          a *= 0.5;
        }
        return v;
      }

      void main() {
        vec2 uv = vUv;
        float f = Fresnel(eyeVector, vNormal);
        
        // Animated noise pattern
        vec2 olduv = uv;
        olduv.y = olduv.y - time * 0.004;
        vec2 p = olduv * 8.0;
        float n = fbm(p + time);
        
        // Gradient effect
        float gradient = dot(normalize(camPos), normalize(vNormal));
        
        // Color mixing with noise
        vec3 color = mix(color0, color1, n + gradient * 0.3);
        color += vec3(f * 0.4); // Stronger Fresnel glow
        
        // Edge glow effect
        float edge = smoothstep(0.0, 0.15, min(min(uv.x, 1.0 - uv.x), min(uv.y, 1.0 - uv.y)));
        color += (1.0 - edge) * color0 * 0.6;
        
        // Sparkle effect
        vec2 sparkleUV = uv;
        sparkleUV.y += time * 0.1;
        sparkleUV.x -= sin(time * 0.1) * 0.5;
        float sparkle = noise(sparkleUV * 4.0 + time * 0.1);
        sparkle = pow(sparkle, 10.0);
        color += vec3(sparkle * 0.3);
        
        gl_FragColor = vec4(color, 0.85 + f * 0.15);
      }
    `;

    // Create card geometry
    const geometry = new window.THREE.PlaneGeometry(20, 30);
    const material = new window.THREE.ShaderMaterial({
      uniforms,
      vertexShader: vert,
      fragmentShader: frag,
      transparent: true,
      side: window.THREE.DoubleSide,
    });

    const card = new window.THREE.Mesh(geometry, material);
    scene.add(card);
    cardMeshRef.current = card;

    // Animation loop - EXACT like original
    const clock = new window.THREE.Clock();
    const animate = () => {
      animationFrameRef.current = requestAnimationFrame(animate);
      
      const elapsed = clock.getElapsedTime();
      uniforms.time.value = elapsed;

      // Update controls (OrbitControls or manual)
      if (controls && typeof controls.update === 'function') {
        controls.update();
      }

      renderer.render(scene, camera);
    };

    animate();

    const handleResize = () => {
      const newWidth = container.offsetWidth;
      const newHeight = container.offsetHeight;
      camera.aspect = newWidth / newHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(newWidth, newHeight);
      uniforms.resolution.value.set(newWidth, newHeight);
    };

    window.addEventListener('resize', handleResize);

    return () => {
      container.removeEventListener('mousemove', handleMouseMove);
      container.removeEventListener('mousedown', handleMouseDown);
      window.removeEventListener('resize', handleResize);
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
      if (renderer.domElement && renderer.domElement.parentNode) {
        renderer.domElement.parentNode.removeChild(renderer.domElement);
      }
      if (controls && controls.dispose) {
        controls.dispose();
      }
      renderer.dispose();
    };
  }, [threeLoaded]);

  return (
    <div ref={containerRef} className={`${styles.ghostCardContainer} ${className || ''}`}>
      <div className={styles.cardContent}>{children}</div>
    </div>
  );
}

