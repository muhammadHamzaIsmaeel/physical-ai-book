// src/components/ChatWidget/ChatWidget.js
import React, { useState, useEffect, useRef } from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import './ChatWidget.css';

// Backend API URL - change this after deploying backend to Render
const API_BASE_URL = process.env.NODE_ENV === 'production'
    ? 'https://ai-book-chatbot-backend.onrender.com'  // Update this with your Render URL
    : 'http://localhost:8000';

const ChatWidget = () => {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState([]);
    const [input, setInput] = useState('');
    const [loading, setLoading] = useState(false);
    const chatBodyRef = useRef(null);

    useEffect(() => {
        if (chatBodyRef.current) {
            chatBodyRef.current.scrollTop = chatBodyRef.current.scrollHeight;
        }
    }, [messages, loading]);

    const handleToggle = () => setIsOpen(!isOpen);

    const handleSend = async () => {
        if (input.trim() === '' || loading) return;

        const userMessage = { role: 'user', content: input };
        const newMessages = [...messages, userMessage];
        setMessages(newMessages);
        setInput('');
        setLoading(true);

        try {
            const response = await fetch(`${API_BASE_URL}/chat`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    message: input,
                    history: messages.slice(-4), // Send last 4 messages for context
                }),
            });

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            if (!response.body) {
                throw new Error("Response body is null");
            }

            const reader = response.body.getReader();
            const decoder = new TextDecoder();
            let done = false;

            let assistantMessage = '';
            setMessages(prev => [...prev, { role: 'assistant', content: '' }]);

            while (!done) {
                const { value, done: readerDone } = await reader.read();
                done = readerDone;
                const chunk = decoder.decode(value, { stream: true });

                // Process Server-Sent Events
                const lines = chunk.split('\n\n').filter(line => line.startsWith('data:'));
                for (const line of lines) {
                    const data = line.substring(5).trim();
                    if (data === '[DONE]') {
                        done = true;
                        break;
                    }
                    try {
                        const parsed = JSON.parse(data);
                        assistantMessage += parsed.content;
                        setMessages(prev => {
                            const lastMsg = prev[prev.length - 1];
                            if (lastMsg.role === 'assistant') {
                                return [...prev.slice(0, -1), { ...lastMsg, content: assistantMessage }];
                            }
                            return [...prev, { role: 'assistant', content: assistantMessage }];
                        });
                    } catch (e) {
                        console.error('Failed to parse SSE data:', data);
                    }
                }
            }
        } catch (error) {
            console.error('API call failed:', error);
            const errorMessage = "Sorry, I couldn't connect to the AI assistant. Please check your connection or try again later.";
            setMessages(prev => {
                const lastMsg = prev[prev.length - 1];
                if (lastMsg.role === 'assistant' && lastMsg.content === '') {
                    return [...prev.slice(0, -1), { ...lastMsg, content: errorMessage }];
                }
                return [...prev, { role: 'assistant', content: errorMessage }];
            });
        } finally {
            setLoading(false);
        }
    };

    const handleKeyPress = (e) => {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            handleSend();
        }
    };

    return (
        <div className="chat-widget-container">
            {isOpen && (
                <div className="chat-window">
                    <div className="chat-header">
                        <h2>AI Book Assistant</h2>
                        <button onClick={handleToggle} className="close-btn" aria-label="Close chat">
                            ×
                        </button>
                    </div>
                    <div className="chat-body" ref={chatBodyRef}>
                        {messages.length === 0 && (
                            <div className="welcome-message">
                                <p>👋 Welcome! Ask me anything about Physical AI and Humanoid Robotics.</p>
                                <p className="example-questions">Try asking:</p>
                                <ul>
                                    <li>"What is URDF?"</li>
                                    <li>"How do I set up Isaac Sim?"</li>
                                    <li>"Explain ROS 2 nodes"</li>
                                </ul>
                            </div>
                        )}
                        {messages.map((msg, index) => (
                            <div key={index} className={`message ${msg.role}`}>
                                {msg.role === 'user' ? (
                                    <div className="message-content">{msg.content}</div>
                                ) : (
                                    <div className="message-content markdown-content">
                                        <ReactMarkdown
                                            remarkPlugins={[remarkGfm]}
                                            components={{
                                                a: ({node, ...props}) => (
                                                    <a {...props} target="_blank" rel="noopener noreferrer" />
                                                ),
                                                code: ({node, inline, ...props}) => (
                                                    inline ?
                                                        <code className="inline-code" {...props} /> :
                                                        <code className="code-block" {...props} />
                                                )
                                            }}
                                        >
                                            {msg.content}
                                        </ReactMarkdown>
                                    </div>
                                )}
                            </div>
                        ))}
                        {loading && messages[messages.length - 1]?.role !== 'assistant' && (
                            <div className="message assistant">
                                <div className="typing-indicator">
                                    <span></span>
                                    <span></span>
                                    <span></span>
                                </div>
                            </div>
                        )}
                    </div>
                    <div className="chat-footer">
                        <input
                            type="text"
                            value={input}
                            onChange={(e) => setInput(e.target.value)}
                            onKeyPress={handleKeyPress}
                            placeholder="Ask a question..."
                            disabled={loading}
                            aria-label="Message input"
                        />
                        <button
                            onClick={handleSend}
                            disabled={loading || input.trim() === ''}
                            aria-label="Send message"
                            className="send-button"
                        >
                            {loading ? '...' : '→'}
                        </button>
                    </div>
                </div>
            )}
            <button
                onClick={handleToggle}
                className="chat-bubble"
                aria-label={isOpen ? "Close chat" : "Open chat"}
            >
                {isOpen ? '×' : '💬'}
            </button>
        </div>
    );
};

export default ChatWidget;
