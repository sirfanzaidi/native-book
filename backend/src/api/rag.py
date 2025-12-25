"""
RAG API endpoints for query processing and session management.

This module implements the core RAG API endpoints:
- POST /api/rag/query: Submit queries (full-book or selected-text mode)
- POST /api/rag/sessions/selected-text: Create selected-text sessions
"""

import time
from typing import Any
from uuid import UUID

from fastapi import APIRouter, HTTPException, status
from fastapi.responses import JSONResponse

from src.config.logging import get_logger
from src.models.rag_models import (
    QueryRequest,
    QueryResponse,
    SelectedTextSessionRequest,
    SelectedTextSessionResponse,
    ErrorResponse,
)
from src.services.retrieval import RAGRetriever

logger = get_logger(__name__)

# Create API router
router = APIRouter(prefix="/api/rag", tags=["rag"])


def get_services():
    """Get global service instances from main module."""
    import src.main as main
    return (
        main.vector_store,
        main.embedding_service,
        main.openrouter_client,
        main.database_service,
    )


@router.post(
    "/query",
    response_model=QueryResponse,
    responses={
        400: {"model": ErrorResponse},
        429: {"model": ErrorResponse},
        500: {"model": ErrorResponse},
    },
)
async def query_rag(request: QueryRequest) -> QueryResponse:
    """
    Submit a query to the RAG chatbot.

    Supports two modes:
    - full_book: Retrieves from all indexed book chapters
    - selected_text: Retrieves only from user-selected text (requires session_id)

    Args:
        request: Query request with text, mode, and optional session_id

    Returns:
        QueryResponse with answer, sources, and latency

    Raises:
        HTTPException: On validation errors, rate limits, or service failures
    """
    start_time = time.time()

    # Get global service instances
    vector_store, embedding_service, openrouter_client, database_service = get_services()

    try:
        # Validate empty query
        if not request.query or not request.query.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Please enter a question.",
            )

        # Validate session for selected_text mode
        if request.mode == "selected_text" and not request.session_id:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="session_id is required for selected_text mode",
            )

        # Check session exists and not expired for selected_text mode
        if request.mode == "selected_text":
            session = await database_service.get_session(request.session_id)
            if not session:
                raise HTTPException(
                    status_code=status.HTTP_404_NOT_FOUND,
                    detail="Session not found or expired",
                )

        # Generate query embedding
        query_vector = embedding_service.embed_text(request.query)

        # Initialize retriever
        retriever = RAGRetriever(
            vector_store=vector_store,
            embedding_service=embedding_service,
        )

        # Retrieve relevant chunks
        if request.mode == "full_book":
            chunks = await retriever.retrieve_full_book(
                query_vector=query_vector,
                top_k=5,
                score_threshold=0.50,
            )
        else:  # selected_text mode
            chunks = await retriever.retrieve_selected_text(
                query_vector=query_vector,
                session_id=request.session_id,
                top_k=5,
                score_threshold=0.40,
            )

        # Handle no results
        if not chunks:
            no_results_message = (
                "I don't have information about that topic in this book"
                if request.mode == "full_book"
                else "I don't have information about that topic in the selected text"
            )
            return QueryResponse(
                answer=no_results_message,
                sources=[],
                latency_ms=int((time.time() - start_time) * 1000),
                query_mode=request.mode,
            )

        # Format context from retrieved chunks
        context = retriever.format_context(chunks)

        # Generate answer using OpenRouter
        system_prompt = (
            "You are a helpful AI assistant answering questions about AI and robotics. "
            "Answer using ONLY the provided context. "
            "If the context doesn't contain enough information to answer the question, "
            "say 'I don't have information about that topic in this book.'"
        )

        response_data = await openrouter_client.chat_completion(
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {request.query}"},
            ],
            model="anthropic/claude-3-haiku",
            temperature=0.7,
            max_tokens=500,
        )

        answer = response_data["content"]

        # Extract source references
        sources = retriever.extract_sources(chunks)

        # Calculate latency
        latency_ms = int((time.time() - start_time) * 1000)

        logger.info(
            "query_processed",
            mode=request.mode,
            chunks_retrieved=len(chunks),
            latency_ms=latency_ms,
        )

        return QueryResponse(
            answer=answer,
            sources=sources,
            latency_ms=latency_ms,
            model_used="anthropic/claude-3-haiku",
            query_mode=request.mode,
        )

    except HTTPException:
        raise

    except Exception as e:
        logger.error(
            "query_processing_failed",
            error=str(e),
            error_type=type(e).__name__,
            mode=request.mode,
        )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="The chatbot is temporarily unavailable. Please try again in a few moments.",
        ) from e


@router.post(
    "/sessions/selected-text",
    response_model=SelectedTextSessionResponse,
    responses={
        400: {"model": ErrorResponse},
        500: {"model": ErrorResponse},
    },
)
async def create_selected_text_session(
    request: SelectedTextSessionRequest
) -> SelectedTextSessionResponse:
    """
    Create a selected-text session for focused queries.

    Indexes the user-selected text as vector embeddings and creates a session
    that expires after 1 hour.

    Args:
        request: Selected text session request

    Returns:
        SelectedTextSessionResponse with session_id and expiry

    Raises:
        HTTPException: On validation or indexing failures
    """
    # Get global service instances
    vector_store, embedding_service, _, database_service = get_services()

    try:
        # Create session in database
        session = await database_service.create_selected_text_session(
            selected_text=request.selected_text
        )

        # Chunk the selected text
        from src.services.chunker import chunk_text
        chunks = chunk_text(
            text=request.selected_text,
            chunk_size=512,
            overlap=100,
        )

        # Generate embeddings and store in Qdrant
        from uuid import uuid4
        points = []
        for idx, chunk in enumerate(chunks):
            embedding = embedding_service.embed_text(chunk)
            point = {
                "id": str(uuid4()),
                "vector": embedding,
                "payload": {
                    "content_text": chunk[:500],
                    "query_mode": "selected_text",
                    "is_selected": True,
                    "selected_session_id": str(session.session_id),
                    "chunk_order": idx,
                    "metadata": request.metadata or {},
                },
            }
            points.append(point)

        # Upsert to Qdrant
        await vector_store.upsert_vectors(points)

        logger.info(
            "selected_text_session_created",
            session_id=str(session.session_id),
            chunks_indexed=len(chunks),
        )

        return SelectedTextSessionResponse(
            session_id=session.session_id,
            expires_at=session.expires_at.isoformat(),
            chunks_indexed=len(chunks),
        )

    except Exception as e:
        logger.error(
            "selected_text_session_creation_failed",
            error=str(e),
            error_type=type(e).__name__,
        )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create selected text session",
        ) from e
