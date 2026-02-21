import React, { useEffect, useRef } from 'react';
import './Radar.css';

const Radar = ({ range, maxRange = 3.0, fov = 0.5 }) => {
  const canvasRef = useRef(null);
  const rangeRef = useRef(range);

  // Update ref when prop changes
  useEffect(() => {
    rangeRef.current = range;
  }, [range]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    const centerX = width / 2;
    const centerY = height - 20; // Leave some space at bottom
    const radius = Math.min(width / 2, height - 40);

    let animationId;

    const draw = () => {
      const currentRange = rangeRef.current;

      ctx.clearRect(0, 0, width, height);

      // Draw background sectors
      ctx.beginPath();
      ctx.arc(centerX, centerY, radius, Math.PI, 2 * Math.PI);
      ctx.fillStyle = '#0f172a';
      ctx.fill();

      // Draw grid lines
      ctx.strokeStyle = '#334155';
      ctx.lineWidth = 1;
      
      // Arcs
      for (let i = 1; i <= 3; i++) {
        ctx.beginPath();
        ctx.arc(centerX, centerY, radius * (i / 3), Math.PI, 2 * Math.PI);
        ctx.stroke();
      }

      // Lines
      const angles = [Math.PI, Math.PI * 1.25, Math.PI * 1.5, Math.PI * 1.75, 2 * Math.PI];
      angles.forEach(angle => {
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(
          centerX + radius * Math.cos(angle),
          centerY + radius * Math.sin(angle)
        );
        ctx.stroke();
      });

      // Draw object detection
      if (currentRange !== null && currentRange < maxRange) {
        const objectRadius = (currentRange / maxRange) * radius;
        
        // Dynamic color based on distance
        let color = '#22c55e'; // Green
        if (currentRange < 0.5) color = '#ef4444'; // Red
        else if (currentRange < 1.0) color = '#eab308'; // Yellow

        ctx.beginPath();
        ctx.arc(centerX, centerY, objectRadius, Math.PI * 1.25, Math.PI * 1.75);
        ctx.strokeStyle = color;
        ctx.lineWidth = 4;
        ctx.stroke();

        // Glow effect
        ctx.beginPath();
        ctx.arc(centerX, centerY, objectRadius, Math.PI * 1.25, Math.PI * 1.75);
        ctx.strokeStyle = color;
        ctx.lineWidth = 10;
        ctx.globalAlpha = 0.3;
        ctx.stroke();
        ctx.globalAlpha = 1.0;

        // Text
        ctx.fillStyle = '#ffffff';
        ctx.font = '16px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(`${currentRange.toFixed(2)}m`, centerX, centerY - 10);
      } else {
        ctx.fillStyle = '#64748b';
        ctx.font = '14px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('No Object', centerX, centerY - 10);
      }
      
      // Draw scanning line animation
      const time = Date.now() / 1000;
      const scanAngle = Math.PI + (Math.sin(time * 2) + 1) * 0.5 * Math.PI; // Oscillate between PI and 2PI
      
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.lineTo(
        centerX + radius * Math.cos(scanAngle),
        centerY + radius * Math.sin(scanAngle)
      );
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.5)';
      ctx.lineWidth = 2;
      ctx.stroke();

      animationId = requestAnimationFrame(draw);
    };

    draw();

    return () => cancelAnimationFrame(animationId);
  }, [maxRange]); // Only re-run if canvas dimensions/maxRange change

  return (
    <div className="radar-container">
      <canvas 
        ref={canvasRef} 
        width={300} 
        height={200} 
        className="radar-canvas"
      />
    </div>
  );
};

export default Radar;
