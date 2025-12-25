/**
 * RAG Chatbot Widget for AI/Robotics Book
 *
 * Provides an interactive chatbot interface for querying book content
 * using the RAG (Retrieval-Augmented Generation) backend API.
 */

(function() {
  'use strict';

  // Configuration
  const API_BASE_URL = 'http://localhost:8080';
  const API_ENDPOINT = `${API_BASE_URL}/api/rag/query`;

  // State
  let isOpen = false;
  let isLoading = false;

  /**
   * Initialize the chatbot widget
   */
  function initChatbot() {
    // Create chatbot HTML structure
    const chatbotHTML = `
      <div id="rag-chatbot-container" class="rag-chatbot-closed">
        <!-- Toggle Button -->
        <button id="rag-chatbot-toggle" class="rag-chatbot-toggle" aria-label="Toggle chatbot">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        </button>

        <!-- Chat Widget -->
        <div id="rag-chatbot-widget" class="rag-chatbot-widget">
          <!-- Header -->
          <div class="rag-chatbot-header">
            <h3>📚 Ask About This Book</h3>
            <button id="rag-chatbot-close" class="rag-chatbot-close" aria-label="Close chatbot">
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <line x1="18" y1="6" x2="6" y2="18"></line>
                <line x1="6" y1="6" x2="18" y2="18"></line>
              </svg>
            </button>
          </div>

          <!-- Messages Area -->
          <div id="rag-chatbot-messages" class="rag-chatbot-messages">
            <div class="rag-chatbot-message rag-chatbot-message-bot">
              <div class="rag-chatbot-message-content">
                👋 Hi! I can answer questions about this AI and Robotics book. Try asking about ROS2, Gazebo, NVIDIA Isaac, or LLMs for robotics!
              </div>
            </div>
          </div>

          <!-- Input Area -->
          <div class="rag-chatbot-input-container">
            <form id="rag-chatbot-form" class="rag-chatbot-form">
              <input
                type="text"
                id="rag-chatbot-input"
                class="rag-chatbot-input"
                placeholder="Ask a question..."
                autocomplete="off"
                maxlength="2000"
                disabled
              />
              <button
                type="submit"
                id="rag-chatbot-submit"
                class="rag-chatbot-submit"
                disabled
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <line x1="22" y1="2" x2="11" y2="13"></line>
                  <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
                </svg>
              </button>
            </form>
          </div>
        </div>
      </div>
    `;

    // Inject into page
    document.body.insertAdjacentHTML('beforeend', chatbotHTML);

    // Setup event listeners
    setupEventListeners();

    // Enable input after initialization
    document.getElementById('rag-chatbot-input').disabled = false;
    document.getElementById('rag-chatbot-submit').disabled = false;
  }

  /**
   * Setup event listeners for chatbot interactions
   */
  function setupEventListeners() {
    const toggleBtn = document.getElementById('rag-chatbot-toggle');
    const closeBtn = document.getElementById('rag-chatbot-close');
    const form = document.getElementById('rag-chatbot-form');

    toggleBtn.addEventListener('click', toggleChatbot);
    closeBtn.addEventListener('click', closeChatbot);
    form.addEventListener('submit', handleSubmit);
  }

  /**
   * Toggle chatbot open/closed
   */
  function toggleChatbot() {
    const container = document.getElementById('rag-chatbot-container');
    isOpen = !isOpen;

    if (isOpen) {
      container.classList.remove('rag-chatbot-closed');
      container.classList.add('rag-chatbot-open');
      document.getElementById('rag-chatbot-input').focus();
    } else {
      container.classList.remove('rag-chatbot-open');
      container.classList.add('rag-chatbot-closed');
    }
  }

  /**
   * Close chatbot
   */
  function closeChatbot() {
    const container = document.getElementById('rag-chatbot-container');
    isOpen = false;
    container.classList.remove('rag-chatbot-open');
    container.classList.add('rag-chatbot-closed');
  }

  /**
   * Handle form submission
   */
  async function handleSubmit(event) {
    event.preventDefault();

    if (isLoading) return;

    const input = document.getElementById('rag-chatbot-input');
    const query = input.value.trim();

    if (!query) return;

    // Add user message
    addMessage(query, 'user');

    // Clear input
    input.value = '';

    // Show loading state
    setLoading(true);
    const loadingMessageId = addLoadingMessage();

    try {
      // Call API
      const response = await fetch(API_ENDPOINT, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: query,
          mode: 'full_book'
        })
      });

      // Remove loading message
      removeMessage(loadingMessageId);

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || errorData.message || 'Failed to get response');
      }

      const data = await response.json();

      // Add bot response
      addBotResponse(data);

    } catch (error) {
      console.error('Chatbot error:', error);
      removeMessage(loadingMessageId);
      addMessage(
        `⚠️ Sorry, I encountered an error: ${error.message}. Please try again.`,
        'bot'
      );
    } finally {
      setLoading(false);
    }
  }

  /**
   * Add a message to the chat
   */
  function addMessage(content, type) {
    const messagesContainer = document.getElementById('rag-chatbot-messages');
    const messageId = `msg-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    const messageHTML = `
      <div id="${messageId}" class="rag-chatbot-message rag-chatbot-message-${type}">
        <div class="rag-chatbot-message-content">${escapeHTML(content)}</div>
      </div>
    `;

    messagesContainer.insertAdjacentHTML('beforeend', messageHTML);
    scrollToBottom();

    return messageId;
  }

  /**
   * Add loading message
   */
  function addLoadingMessage() {
    const messagesContainer = document.getElementById('rag-chatbot-messages');
    const messageId = `loading-${Date.now()}`;

    const loadingHTML = `
      <div id="${messageId}" class="rag-chatbot-message rag-chatbot-message-bot">
        <div class="rag-chatbot-message-content">
          <div class="rag-chatbot-loading">
            <span></span>
            <span></span>
            <span></span>
          </div>
        </div>
      </div>
    `;

    messagesContainer.insertAdjacentHTML('beforeend', loadingHTML);
    scrollToBottom();

    return messageId;
  }

  /**
   * Add bot response with sources
   */
  function addBotResponse(data) {
    const messagesContainer = document.getElementById('rag-chatbot-messages');
    const messageId = `msg-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    // Build sources HTML
    let sourcesHTML = '';
    if (data.sources && data.sources.length > 0) {
      sourcesHTML = '<div class="rag-chatbot-sources"><strong>Sources:</strong><ul>';

      // Remove duplicates based on chapter_title
      const uniqueSources = [];
      const seenTitles = new Set();

      for (const source of data.sources) {
        if (!seenTitles.has(source.chapter_title)) {
          seenTitles.add(source.chapter_title);
          uniqueSources.push(source);
        }
      }

      for (const source of uniqueSources) {
        const score = Math.round(source.relevance_score * 100);
        sourcesHTML += `
          <li>
            <a href="${escapeHTML(source.source_url)}" target="_blank" rel="noopener noreferrer">
              ${escapeHTML(source.chapter_title)}
            </a>
            <span class="rag-chatbot-score">${score}% relevant</span>
          </li>
        `;
      }
      sourcesHTML += '</ul></div>';
    }

    // Build latency info
    const latencyInfo = data.latency_ms
      ? `<div class="rag-chatbot-latency">⚡ ${(data.latency_ms / 1000).toFixed(2)}s</div>`
      : '';

    const messageHTML = `
      <div id="${messageId}" class="rag-chatbot-message rag-chatbot-message-bot">
        <div class="rag-chatbot-message-content">
          ${escapeHTML(data.answer)}
          ${sourcesHTML}
          ${latencyInfo}
        </div>
      </div>
    `;

    messagesContainer.insertAdjacentHTML('beforeend', messageHTML);
    scrollToBottom();
  }

  /**
   * Remove a message
   */
  function removeMessage(messageId) {
    const message = document.getElementById(messageId);
    if (message) {
      message.remove();
    }
  }

  /**
   * Set loading state
   */
  function setLoading(loading) {
    isLoading = loading;
    const input = document.getElementById('rag-chatbot-input');
    const submit = document.getElementById('rag-chatbot-submit');

    input.disabled = loading;
    submit.disabled = loading;
  }

  /**
   * Scroll messages to bottom
   */
  function scrollToBottom() {
    const messagesContainer = document.getElementById('rag-chatbot-messages');
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }

  /**
   * Escape HTML to prevent XSS
   */
  function escapeHTML(str) {
    const div = document.createElement('div');
    div.textContent = str;
    return div.innerHTML;
  }

  // Initialize when DOM is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initChatbot);
  } else {
    initChatbot();
  }

})();
