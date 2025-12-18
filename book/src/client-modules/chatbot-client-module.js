// book/src/client-modules/chatbot-client-module.js
import React, { useState } from 'react';
import ChatbotUI from '../components/ChatbotUI';
import SelectionHandler from '../components/SelectionHandler';

// This is the Root component that will wrap the entire Docusaurus app
// and allow us to inject our client-side components.
export default function Root({ children }) {
  const [selectedText, setSelectedText] = useState('');

  const handleTextSelect = (text) => {
    setSelectedText(text);
  };

  // Render the components here
  return (
    <>
      {children}
      <SelectionHandler onTextSelect={handleTextSelect} />
      <ChatbotUI selectedText={selectedText} />
    </>
  );
}