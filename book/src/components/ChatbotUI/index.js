// book/src/components/ChatbotUI/index.js
import React, { useState, useEffect } from 'react';
import { postQuery } from '../../services/api';
import styles from './styles.module.css'; // We will create this later

function ChatbotUI({ selectedText: propSelectedText }) {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [currentInput, setCurrentInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [errorMessage, setErrorMessage] = useState('');
  const [selectedTextContext, setSelectedTextContext] = useState('');
  const [useSelectedTextAsContext, setUseSelectedTextAsContext] = useState(false);

  useEffect(() => {
    if (propSelectedText) {
      setSelectedTextContext(propSelectedText);
    }
  }, [propSelectedText]);

  const toggleChat = () => {
    setIsChatOpen(!isChatOpen);
  };

  const handleInputChange = (event) => {
    setCurrentInput(event.target.value);
  };

  const sendMessage = async () => {
    if (currentInput.trim() === '' && !useSelectedTextAsContext) return;

    const queryPayload = { query: currentInput };
    if (useSelectedTextAsContext && selectedTextContext) {
      queryPayload.selected_text_context = selectedTextContext;
    }

    const userMessage = { text: currentInput, sender: 'user', context: useSelectedTextAsContext ? selectedTextContext : null };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setCurrentInput('');
    setIsLoading(true);
    setErrorMessage('');
    setUseSelectedTextAsContext(false); // Reset after sending

    try {
      const response = await postQuery(queryPayload);
      setMessages((prevMessages) => [
        ...prevMessages,
        { text: response.answer, sender: 'agent', sources: response.sources },
      ]);
    } catch (error) {
      setErrorMessage('Failed to get a response. Please try again.');
      console.error('Chatbot error:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (event) => {
    if (event.key === 'Enter') {
      sendMessage();
    }
  };

  const handleUseContext = () => {
    setUseSelectedTextAsContext(!useSelectedTextAsContext);
  };

  const clearSelectedContext = () => {
    setSelectedTextContext('');
    setUseSelectedTextAsContext(false);
  };

  // Basic UI structure, styling will be added later
  return (
    <div className={styles.chatbotContainer}>
      <button className={styles.chatToggleButton} onClick={toggleChat}>
        {isChatOpen ? 'Close Chat' : 'Open Chat'}
      </button>

      {isChatOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>Ask the Book</h3>
            <button className={styles.closeButton} onClick={toggleChat}>X</button>
          </div>
          <div className={styles.messagesDisplay}>
            {messages.map((msg, index) => (
              <div key={index} className={`${styles.message} ${styles[msg.sender]}`}>
                <p>{msg.text}</p>
                {msg.context && <p className={styles.contextUsed}>Context used: "{msg.context}"</p>}
                {msg.sources && msg.sources.length > 0 && (
                  <div className={styles.sources}>
                    <h4>Sources:</h4>
                    <ul>
                      {msg.sources.map((source, srcIndex) => (
                        <li key={srcIndex}>
                          <a href={source.url} target="_blank" rel="noopener noreferrer">
                            {source.title || source.url}
                          </a>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            ))}
            {isLoading && <div className={styles.loading}>Agent is thinking...</div>}
            {errorMessage && <div className={styles.error}>{errorMessage}</div>}
          </div>
          {selectedTextContext && (
            <div className={styles.selectedContextPreview}>
              <p>Selected Text: "{selectedTextContext}"</p>
              <label>
                <input
                  type="checkbox"
                  checked={useSelectedTextAsContext}
                  onChange={handleUseContext}
                />
                Use as context for next query
              </label>
              <button onClick={clearSelectedContext}>Clear Context</button>
            </div>
          )}
          <div className={styles.chatInputContainer}>
            <input
              type="text"
              value={currentInput}
              onChange={handleInputChange}
              onKeyPress={handleKeyPress}
              placeholder="Type your question..."
              className={styles.chatInput}
              disabled={isLoading}
            />
            <button onClick={sendMessage} className={styles.sendButton} disabled={isLoading}>
              Send
            </button>
          </div>
        </div>
      )}
    </div>
  );
}

export default ChatbotUI;
