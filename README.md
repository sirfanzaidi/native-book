---
title: RAG Chatbot Backend
emoji: 🤖
colorFrom: purple
colorTo: blue
sdk: docker
pinned: false
app_port: 7860
---

# RAG Chatbot Backend

FastAPI backend service for the Integrated RAG Chatbot that enables interactive querying of AI/Robotics book content.

## Features

- **Full-book queries**: Search across all book chapters
- **Selected-text queries**: Ask questions about specific highlighted passages
- **Source references**: All answers include clickable source links
- **Vector search**: Semantic search using Qdrant and sentence-transformers
- **LLM generation**: Answer generation via OpenRouter API

## Tech Stack

- **Web Framework**: FastAPI + uvicorn
- **Vector Store**: Qdrant Cloud (free tier)
- **Database**: Neon Serverless Postgres
- **LLM**: OpenRouter API (mistralai/devstral-2512:free)
- **Embeddings**: sentence-transformers/all-MiniLM-L6-v2 (384-dim)
- **Python**: 3.11+

## Quick Start

### 1. Prerequisites

- Python 3.11 or higher
- API keys for:
  - Qdrant Cloud (free tier)
  - Neon Serverless Postgres
  - OpenRouter

### 2. Environment Configuration

This backend requires the following environment variables to be set in Hugging Face Spaces secrets:

```
OPENROUTER_API_KEY=your_openrouter_api_key
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
OPENROUTER_MODEL=mistralai/devstral-2512:free

QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_embeddings

NEON_DATABASE_URL=your_neon_connection_string

CORS_ALLOW_ORIGINS=https://sirfanzaidi.github.io/native-book,http://localhost:3000,http://localhost:3001
```

### 3. API Endpoints

#### Health Check
```
GET /health
```

#### RAG Query
```
POST /api/rag/query

Request Body:
{
  "query": "What is ROS 2?",
  "mode": "full_book",
  "session_id": "optional-session-id"
}

Response:
{
  "answer": "ROS 2 is...",
  "sources": [
    {
      "chapter_title": "Introduction to ROS 2",
      "source_url": "/module-1-ros2/01-introduction",
      "relevance_score": 0.85
    }
  ],
  "latency_ms": 3500,
  "session_id": "session-123"
}
```

## Deployment to Hugging Face Spaces

This backend is configured for deployment to Hugging Face Spaces using Docker.

### Configuration
- Port: 7860 (required by Hugging Face Spaces)
- SDK: Docker
- The Dockerfile is configured to pre-download embedding models for faster startup

### Build Process
1. Docker downloads Python dependencies
2. Downloads sentence-transformers model during build
3. Copies source code from backend/src/
4. Starts FastAPI server on port 7860

## Deployment

### GitHub Pages (Current)
The site is currently deployed to GitHub Pages at: https://sirfanzaidi.github.io/native-book/

### Vercel (New Option)
The site can also be deployed to Vercel. See [VERCEL_README.md](VERCEL_README.md) for deployment instructions.

## License

See main project LICENSE file.