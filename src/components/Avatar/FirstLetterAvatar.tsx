import React from 'react';
import { generateAvatarColor } from '@site/src/utils/avatar-generator';

interface FirstLetterAvatarProps {
  letter: string;
  size?: number;
  className?: string;
}

/**
 * Component that generates and displays an SVG avatar with the first letter of user's name
 * Handles edge cases: empty names, special characters, non-ASCII characters
 */
export function FirstLetterAvatar({ 
  letter, 
  size = 40, 
  className = '' 
}: FirstLetterAvatarProps) {
  const color = generateAvatarColor(letter);
  const fontSize = Math.floor(size * 0.5);
  
  return (
    <svg
      width={size}
      height={size}
      viewBox={`0 0 ${size} ${size}`}
      className={className}
      style={{ borderRadius: '50%' }}
    >
      <circle
        cx={size / 2}
        cy={size / 2}
        r={size / 2}
        fill={color}
      />
      <text
        x={size / 2}
        y={size / 2}
        fontSize={fontSize}
        fontWeight="bold"
        fill="white"
        textAnchor="middle"
        dominantBaseline="central"
        fontFamily="system-ui, -apple-system, sans-serif"
      >
        {letter}
      </text>
    </svg>
  );
}

