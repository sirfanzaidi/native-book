"""
FastAPI application entry point for RAG Chatbot backend.

This module initializes the FastAPI application with CORS configuration,
error handlers, and API routes.
"""

import os
from contextlib import asynccontextmanager
from typing import Any, AsyncGenerator, Optional

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from dotenv import load_dotenv

from src.config.logging import get_logger, setup_logging
from src.core.database import DatabaseService
from src.core.vector_store import QdrantVectorStore
from src.services.embeddings import EmbeddingService
from src.services.openrouter_client import OpenRouterClient

# Load environment variables
load_dotenv()

# Setup logging
setup_logging(log_level=os.getenv("LOG_LEVEL", "INFO"))
logger = get_logger(__name__)

# Global service instances (initialized in lifespan)
vector_store: Optional[QdrantVectorStore] = None
database_service: Optional[DatabaseService] = None
embedding_service: Optional[EmbeddingService] = None
openrouter_client: Optional[OpenRouterClient] = None


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """
    Application lifespan context manager.

    Handles startup and shutdown events for initializing/cleaning up
    database connections, vector store clients, and other resources.

    Args:
        app: FastAPI application instance

    Yields:
        None during application runtime
    """
    global vector_store, database_service, embedding_service, openrouter_client

    # Startup
    logger.info("application_startup", message="Initializing RAG chatbot backend")

    try:
        # Initialize Qdrant vector store
        vector_store = QdrantVectorStore()
        logger.info("qdrant_initialized")

        # Initialize Neon Postgres database
        database_service = DatabaseService()
        await database_service.connect()
        logger.info("database_initialized")

        # Initialize embedding service
        embedding_service = EmbeddingService()
        logger.info("embedding_service_initialized")

        # Initialize OpenRouter client
        openrouter_client = OpenRouterClient()
        logger.info("openrouter_client_initialized")

        # Create Qdrant collection if not exists
        await vector_store.create_collection()

        logger.info("application_startup_complete", message="All services initialized")

    except Exception as e:
        logger.error(
            "application_startup_failed",
            error=str(e),
            error_type=type(e).__name__,
        )
        raise

    yield

    # Shutdown
    logger.info("application_shutdown", message="Shutting down RAG chatbot backend")

    try:
        # Close database connections
        if database_service:
            await database_service.close()

        # Close OpenRouter client
        if openrouter_client:
            await openrouter_client.close()

        logger.info("application_shutdown_complete", message="All services cleaned up")

    except Exception as e:
        logger.error(
            "application_shutdown_error",
            error=str(e),
            error_type=type(e).__name__,
        )


# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for AI/Robotics book content",
    version="0.1.0",
    lifespan=lifespan,
)

# Configure CORS
cors_origins = os.getenv("CORS_ALLOW_ORIGINS", "http://localhost:3000").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=os.getenv("CORS_ALLOW_CREDENTIALS", "true").lower() == "true",
    allow_methods=os.getenv("CORS_ALLOW_METHODS", "GET,POST,OPTIONS").split(","),
    allow_headers=["*"],
)

# Register API routers
from src.api import rag
app.include_router(rag.router)


@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception) -> JSONResponse:
    """
    Global exception handler for unhandled errors.

    Args:
        request: The incoming request
        exc: The exception that was raised

    Returns:
        JSONResponse with error details
    """
    logger.error(
        "unhandled_exception",
        error=str(exc),
        error_type=type(exc).__name__,
        path=request.url.path,
    )

    return JSONResponse(
        status_code=500,
        content={
            "error": "Internal server error",
            "message": "An unexpected error occurred. Please try again later.",
        },
    )


@app.get("/")
async def root() -> dict[str, str]:
    """
    Root endpoint returning basic API information.

    Returns:
        dict: API name and version
    """
    return {
        "name": "RAG Chatbot API",
        "version": "0.1.0",
        "status": "operational",
    }


@app.get("/health")
async def health_check() -> dict[str, Any]:
    """
    Health check endpoint with service status.

    Returns:
        dict: Service health status for all components

    Note:
        Checks Qdrant, Neon, and OpenRouter connectivity.
    """
    health_status = {
        "status": "healthy",
        "service": "rag-chatbot-backend",
        "components": {},
    }

    # Check Qdrant
    if vector_store:
        qdrant_healthy = await vector_store.health_check()
        health_status["components"]["qdrant"] = (
            "healthy" if qdrant_healthy else "unhealthy"
        )
    else:
        health_status["components"]["qdrant"] = "not_initialized"

    # Check Neon Postgres
    if database_service:
        db_healthy = await database_service.health_check()
        health_status["components"]["database"] = (
            "healthy" if db_healthy else "unhealthy"
        )
    else:
        health_status["components"]["database"] = "not_initialized"

    # Check OpenRouter (skip for now to avoid quota usage)
    if openrouter_client:
        health_status["components"]["openrouter"] = "initialized"
    else:
        health_status["components"]["openrouter"] = "not_initialized"

    # Overall status is unhealthy if any component is unhealthy
    if any(
        status == "unhealthy"
        for status in health_status["components"].values()
    ):
        health_status["status"] = "degraded"

    return health_status


if __name__ == "__main__":
    import uvicorn

    # Support both FASTAPI_* and SERVER_* environment variables
    host = os.getenv("FASTAPI_HOST") or os.getenv("SERVER_HOST", "0.0.0.0")
    port = int(os.getenv("FASTAPI_PORT") or os.getenv("SERVER_PORT", "8000"))
    reload = os.getenv("FASTAPI_RELOAD", "false").lower() == "true"

    logger.info(
        "starting_server",
        host=host,
        port=port,
        reload=reload,
    )

    uvicorn.run(
        "src.main:app",
        host=host,
        port=port,
        reload=reload,
        log_level=os.getenv("LOG_LEVEL", "info").lower(),
    )
