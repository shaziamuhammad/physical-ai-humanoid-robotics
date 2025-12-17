import React, { useState, useEffect } from 'react';
import './Chatbot.css';

const Chatbot = ({ chapterId }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');

  // Function to get selected text from the page
  const getSelectedText = () => {
    const selection = window.getSelection();
    let selectedText = '';

    if (selection && selection.toString()) {
      selectedText = selection.toString().trim();

      // Check if the selection is within our chatbot UI to avoid recursive selection
      const anchorNode = selection.anchorNode;
      const focusNode = selection.focusNode;

      // If selection contains elements from our chatbot UI, ignore it
      if (anchorNode && focusNode) {
        const anchorElement = anchorNode.parentElement ? anchorNode.parentElement.closest('.chatbot-modal, .chatbot-toggle') : null;
        const focusElement = focusNode.parentElement ? focusNode.parentElement.closest('.chatbot-modal, .chatbot-toggle') : null;

        if (anchorElement || focusElement) {
          // Selection includes chatbot UI, so ignore it
          return;
        }
      }

      // Clean the selected text by removing common UI elements
      // Remove emoji buttons and other UI artifacts that might be selected
      selectedText = selectedText.replace(/[📝🗑️×]/g, ' ')
                                .replace(/\s+/g, ' ')  // Normalize whitespace
                                .trim();

      // Remove any text that looks like it contains chatbot UI elements or previous chat messages
      if (selectedText.includes('Selected text:') || selectedText.includes('Mode:')) {
        // This looks like a recursive selection of previous chatbot output
        return;
      }
    }

    // Check if the selected text is meaningful (not just a single word or very short)
    if (selectedText && selectedText.length > 10) {
      setSelectedText(selectedText);
      // Auto-open the chatbot when text is selected
      setIsOpen(true);
    } else if (selectedText) {
      // If text is selected but too short, show a message to the user
      setMessages(prev => [...prev, {
        type: 'bot',
        content: 'The selected text is too short to be useful. Please select more text to ask questions about.',
        timestamp: new Date()
      }]);
    }
  };

  // Add event listener for text selection
  useEffect(() => {
    const handleMouseUp = () => {
      setTimeout(getSelectedText, 0); // Delay to ensure selection is complete
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Clean and validate selected text before sending to backend
    let validSelectedText = selectedText;
    if (selectedText) {
      // Clean the selected text by removing UI elements
      validSelectedText = selectedText.replace(/[📝🗑️×]/g, ' ')
                                     .replace(/\s+/g, ' ')  // Normalize whitespace
                                     .trim();

      if (validSelectedText.length <= 10) {
        // If cleaned text is too short, clear it and warn the user
        validSelectedText = null;
        setMessages(prev => [...prev, {
          type: 'bot',
          content: 'The selected text was too short, so I\'m searching the entire book for your answer.',
          timestamp: new Date()
        }]);
      }
    }

    const userMessage = { type: 'user', content: inputValue, timestamp: new Date() };
    const newMessages = [...messages, userMessage];
    setMessages(newMessages);
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: inputValue,
          selected_text: validSelectedText || null,
          session_id: localStorage.getItem('chat_session_id') || null,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Store session ID for future requests
      if (data.session_id) {
        localStorage.setItem('chat_session_id', data.session_id);
      }

      const botMessage = {
        type: 'bot',
        content: data.answer,
        citations: data.citations,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        type: 'bot',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear selected text after sending
    }
  };

  const embedCurrentChapter = async () => {
    // This would extract the current chapter's content and send it to the backend
    // For now, we'll simulate by getting all text from the main content area
    const contentElement = document.querySelector('.main-wrapper') || document.querySelector('main') || document.querySelector('article');
    if (!contentElement) {
      alert('Could not find chapter content to embed');
      return;
    }

    const chapterText = contentElement.innerText || contentElement.textContent;

    if (chapterText.length < 50) {
      alert('Not enough content found in this chapter');
      return;
    }

    try {
      const response = await fetch('http://localhost:8000/embed', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text: chapterText,
          chapter_id: chapterId || window.location.pathname,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      alert(`Chapter embedded successfully! Processed ${data.chunks_processed} chunks.`);
    } catch (error) {
      console.error('Error embedding chapter:', error);
      alert('Error embedding chapter. Please check the console for details.');
    }
  };

  const toggleChatbot = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      // When opening, clear selected text
      setSelectedText('');
    }
  };

  const clearChat = () => {
    setMessages([]);
  };

  return (
    <>
      {/* Chatbot toggle button */}
      <button
        className={`chatbot-toggle ${isOpen ? 'open' : ''}`}
        onClick={toggleChatbot}
        aria-label={isOpen ? "Close chatbot" : "Open chatbot"}
      >
        {isOpen ? '×' : '💬'}
      </button>

      {/* Chatbot modal */}
      {isOpen && (
        <div className="chatbot-modal">
          <div className="chatbot-header">
            <h3>Book Assistant</h3>
            <div className="chatbot-actions">
              <button onClick={embedCurrentChapter} title="Embed current chapter" className="embed-btn">
                📝
              </button>
              <button onClick={clearChat} title="Clear chat" className="clear-btn">
                🗑️
              </button>
              <button onClick={toggleChatbot} title="Close" className="close-btn">
                ×
              </button>
            </div>
          </div>

          <div className="chatbot-body">
            {selectedText && (
              <div className="selected-text-preview">
                <small>Selected text:</small>
                <p>"{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</p>
                <small>Mode: Selected-text-only</small>
              </div>
            )}

            <div className="chat-messages">
              {messages.length === 0 ? (
                <div className="welcome-message">
                  <p>Hello! I'm your book assistant for Physical AI & Humanoid Robotics.</p>
                  <p>You can:</p>
                  <ul>
                    <li>Ask questions about the book content</li>
                    <li>Select text on the page and ask questions about it</li>
                    <li>Click the 📝 button to embed the current chapter</li>
                  </ul>
                </div>
              ) : (
                messages.map((message, index) => (
                  <div key={index} className={`message ${message.type}`}>
                    <div className="message-content">
                      {message.type === 'bot' && message.citations && message.citations.length > 0 && (
                        <details className="citations">
                          <summary>Citations ({message.citations.length})</summary>
                          <ul>
                            {message.citations.map((citation, idx) => (
                              <li key={idx} className="citation-item">
                                <small>Ch. {citation.chapter_id} (Score: {citation.score.toFixed(2)})</small>
                                <p>{citation.text}</p>
                              </li>
                            ))}
                          </ul>
                        </details>
                      )}
                      <p>{message.content}</p>
                    </div>
                  </div>
                ))
              )}
              {isLoading && (
                <div className="message bot">
                  <div className="message-content">
                    <p>Thinking...</p>
                  </div>
                </div>
              )}
            </div>
          </div>

          <div className="chatbot-footer">
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
              placeholder={selectedText ? "Ask about selected text..." : "Ask a question about the book..."}
              disabled={isLoading}
            />
            <button onClick={sendMessage} disabled={isLoading || !inputValue.trim()}>
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default Chatbot;