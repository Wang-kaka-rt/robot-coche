import React from 'react';
import './InfraredSensor.css';

const InfraredSensor = ({ value }) => {
  // Convert value to binary string, pad with leading zeros to length 3
  // Value is 3 bits: left (bit 2), middle (bit 1), right (bit 0)
  // 1 means line detected (black), 0 means no line (white) - typically
  // However, check sensor logic: usually IR sensor returns 1 for black/line, 0 for white/floor
  // Or vice versa. Let's assume standard: 1 = active/trigger
  
  const left = (value >> 2) & 1;
  const middle = (value >> 1) & 1;
  const right = value & 1;

  return (
    <div className="infrared-container">
      <div className={`infrared-sensor ${left ? 'active' : ''}`} title="Left Sensor">
        L
      </div>
      <div className={`infrared-sensor ${middle ? 'active' : ''}`} title="Middle Sensor">
        M
      </div>
      <div className={`infrared-sensor ${right ? 'active' : ''}`} title="Right Sensor">
        R
      </div>
      <div className="infrared-value">
        Value: {value !== null ? value : '-'}
      </div>
    </div>
  );
};

export default InfraredSensor;
