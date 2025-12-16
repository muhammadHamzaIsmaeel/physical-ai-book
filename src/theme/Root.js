// src/theme/Root.js
import React from 'react';
import ChatWidget from '../components/ChatWidget/ChatWidget';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
