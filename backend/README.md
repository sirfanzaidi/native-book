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
- **LLM**: OpenRouter API (anthropic/claude-3-haiku)
- **Embeddings**: sentence-transformers/all-MiniLM-L6-v2 (384-dim)
- **Python**: 3.11+

## Quick Start

### 1. Prerequisites

- Python 3.11 or higher
- API keys for:
  - Qdrant Cloud (free tier)
  - Neon Serverless Postgres
  - OpenRouter

### 2. Install Dependencies

```bash
cd backend
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Configure Environment

Copy `.env.example` to `.env` and fill in your API keys:

```bash
cp .env.example .env
# Edit .env with your actual credentials
```

Required environment variables:
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Qdrant API key
- `NEON_DATABASE_URL`: Neon Postgres connection string
- `OPENROUTER_API_KEY`: OpenRouter API key

### 4. Index Book Content

Before running the server, index the book content:

```bash
python scripts/index_book_content.py --docs-dir ../docs
```

This will:
- Read all markdown files from `../docs`
- Chunk the content (512-1024 tokens with 20% overlap)
- Generate embeddings using all-MiniLM-L6-v2
- Store in Qdrant vector database

### 5. Run Development Server

```bash
python src/main.py
```

Or with uvicorn directly:

```bash
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at http://localhost:8000

### 6. Verify Health

Check that all services are running:

```bash
curl http://localhost:8000/health
```

Expected response:
```json
{
  "status": "healthy",
  "service": "rag-chatbot-backend",
  "components": {
    "qdrant": "healthy",
    "database": "healthy",
    "openrouter": "initialized"
  }
}
```

## API Endpoints

### Root
- `GET /` - API information

### Health Check
- `GET /health` - Service health status

### RAG Queries (Coming in Phase 3)
- `POST /api/rag/query` - Submit a query (full-book or selected-text mode)
- `POST /api/rag/sessions/selected-text` - Create selected-text session

### Metrics (Coming in Phase 7)
- `GET /metrics/latency` - Performance metrics

## Project Structure

```
backend/
├── src/
│   ├── main.py                 # FastAPI app entry point
│   ├── config/
│   │   └── logging.py          # Structured logging setup
│   ├── core/
│   │   ├── database.py         # Neon Postgres service
│   │   └── vector_store.py     # Qdrant client wrapper
│   ├── services/
│   │   ├── chunker.py          # Text chunking
│   │   ├── embeddings.py       # Embedding generation
│   │   ├── indexer.py          # Content indexing
│   │   ├── metadata_extractor.py  # Markdown metadata extraction
│   │   └── openrouter_client.py   # LLM API client
│   ├── models/                 # Pydantic models (Phase 3)
│   ├── api/                    # API routes (Phase 3)
│   └── middleware/             # Middleware (Phase 6)
├── scripts/
│   └── index_book_content.py   # Indexing script
├── tests/                      # Test suite (Phase 8)
├── requirements.txt            # Python dependencies
├── .env.example                # Environment template
└── README.md                   # This file
```

## Development

### Code Quality

Run code formatters and linters:

```bash
# Format code
black src/

# Type checking
mypy src/

# Lint
ruff src/
```

### Testing (Phase 8)

```bash
# Run all tests
pytest

# With coverage
pytest --cov=src --cov-report=html

# Run specific test
pytest tests/unit/test_embeddings.py
```

## Configuration

See `.env.example` for all configuration options.

Key settings:
- `CHUNK_SIZE_MIN`, `CHUNK_SIZE_MAX`: Text chunking parameters (default: 512-1024 tokens)
- `CHUNK_OVERLAP_PERCENT`: Chunk overlap (default: 20%)
- `TOP_K_RESULTS`: Number of chunks to retrieve (default: 5)
- `SIMILARITY_THRESHOLD`: Minimum similarity score (default: 0.7)
- `RATE_LIMIT_SUSTAINED_QPS`: Rate limit (default: 2 QPS)

## Troubleshooting

### Qdrant connection failed
- Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
- Check Qdrant Cloud dashboard for cluster status
- Ensure cluster is not paused (free tier auto-pauses after inactivity)

### Neon database connection failed
- Verify `NEON_DATABASE_URL` connection string
- Check Neon project status in dashboard
- Ensure database is not suspended

### Embedding model download slow
- First run downloads ~90MB model from HuggingFace
- Model is cached in `~/.cache/huggingface/`
- Use `EMBEDDING_DEVICE=cpu` (default) if no GPU available

### Indexing fails
- Check docs directory path with `--docs-dir`
- Ensure markdown files exist and are readable
- Check Qdrant storage quota (free tier: 1GB)

## Deployment to Hugging Face Spaces

This backend can be deployed to Hugging Face Spaces using Docker for production hosting.

### Hugging Face Space Configuration

The repository includes Hugging Face-compatible files:
- `Dockerfile` - Docker configuration for Hugging Face Spaces
- `.dockerignore` - Excludes unnecessary files from Docker build
- `README.md` (this file) - Space configuration via frontmatter

### Deployment Steps

1. **Create a new Hugging Face Space**:
   - Go to https://huggingface.co/new-space
   - Choose **Docker** as the SDK
   - Set visibility (Public or Private)
   - Create Space

2. **Clone and push backend code**:
   ```bash
   cd backend

   # Initialize git if not already done
   git init

   # Add Hugging Face Space as remote
   git remote add space https://huggingface.co/spaces/YOUR_USERNAME/SPACE_NAME

   # Add files and commit
   git add Dockerfile .dockerignore README.md requirements.txt src/
   git commit -m "Deploy RAG chatbot backend to Hugging Face"

   # Push to Hugging Face
   git push --force space main
   ```

3. **Configure Environment Secrets**:
   - Go to your Space Settings → Repository secrets
   - Add the following required secrets:

   ```
   OPENROUTER_API_KEY=your_openrouter_api_key
   OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
   OPENROUTER_MODEL=mistralai/devstral-2512:free

   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=book_embeddings

   NEON_DATABASE_URL=your_neon_connection_string

   CORS_ALLOW_ORIGINS=https://yourusername.github.io,http://localhost:3000
   ```

4. **Wait for build and deployment**:
   - Hugging Face will automatically build the Docker image
   - Build logs are visible in the Space's "Logs" tab
   - Deployment typically takes 5-10 minutes

5. **Access your deployed API**:
   - Space URL: `https://YOUR_USERNAME-SPACE_NAME.hf.space`
   - API endpoint: `https://YOUR_USERNAME-SPACE_NAME.hf.space/api/rag/query`
   - Health check: `https://YOUR_USERNAME-SPACE_NAME.hf.space/health`

### Important Notes for Hugging Face Deployment

- **Port**: Hugging Face Spaces requires port 7860 (configured in Dockerfile)
- **Host**: Must bind to `0.0.0.0` (configured in Dockerfile CMD)
- **Model Caching**: Embedding model is pre-downloaded during Docker build for faster startup
- **Workers**: Single worker recommended for free tier (memory constraints)
- **Secrets**: Never commit `.env` file - use Hugging Face Space secrets
- **CORS**: Update `CORS_ALLOW_ORIGINS` to include your frontend domain

### Updating Deployed Space

To update your deployed backend:

```bash
# Make code changes
git add .
git commit -m "Update: description of changes"
git push space main
```

Hugging Face will automatically rebuild and redeploy.

### Frontend Configuration

After deployment, update your frontend `chatbot.js` to use the Hugging Face URL:

```javascript
// Production
const API_BASE_URL = 'https://YOUR_USERNAME-SPACE_NAME.hf.space';

// Or use environment-based configuration
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://YOUR_USERNAME-SPACE_NAME.hf.space'
  : 'http://localhost:8000';
```

## License

See main project LICENSE file.
