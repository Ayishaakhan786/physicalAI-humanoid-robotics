// book/src/components/SelectionHandler/index.js
import React, { useEffect, useState } from 'react';

function SelectionHandler({ onTextSelect }) {
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();
      setSelectedText(text);
      if (onTextSelect) {
        onTextSelect(text);
      }
    };

    document.addEventListener('selectionchange', handleSelectionChange);

    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, [onTextSelect]);

  return null; // This component doesn't render anything visible directly
}

export default SelectionHandler;
