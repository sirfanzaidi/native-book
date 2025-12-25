"""
RAG retrieval service for semantic search and context retrieval.

This module provides retrieval functionality for both full-book
and selected-text query modes with metadata filtering.
"""

from typing import Any, Optional
from uuid import UUID

from src.config.logging import get_logger
from src.core.vector_store import QdrantVectorStore
from src.services.embeddings import EmbeddingService

logger = get_logger(__name__)


class RetrievalResult:
    """Container for a single retrieval result."""

    def __init__(
        self,
        text: str,
        score: float,
        chapter_title: str,
        module: str,
        source_url: str,
        chunk_order: int,
    ) -> None:
        """
        Initialize retrieval result.

        Args:
            text: Retrieved chunk text
            score: Similarity score (0-1)
            chapter_title: Chapter name
            module: Module identifier
            source_url: URL to source document
            chunk_order: Chunk position in document
        """
        self.text = text
        self.score = score
        self.chapter_title = chapter_title
        self.module = module
        self.source_url = source_url
        self.chunk_order = chunk_order

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary representation."""
        return {
            "text": self.text,
            "score": self.score,
            "chapter_title": self.chapter_title,
            "module": self.module,
            "source_url": self.source_url,
            "chunk_order": self.chunk_order,
        }


class RAGRetriever:
    """
    Retrieval-Augmented Generation retrieval service.

    Provides semantic search functionality for full-book and
    selected-text query modes.
    """

    def __init__(
        self,
        vector_store: QdrantVectorStore,
        embedding_service: EmbeddingService,
    ) -> None:
        """
        Initialize RAG retriever.

        Args:
            vector_store: Qdrant vector store instance
            embedding_service: Embedding service instance
        """
        self.vector_store = vector_store
        self.embedding_service = embedding_service

        logger.info("rag_retriever_initialized")

    async def retrieve_full_book(
        self,
        query_vector: list[float],
        top_k: int = 5,
        score_threshold: float = 0.50,
    ) -> list[RetrievalResult]:
        """
        Retrieve relevant chunks from full book content.

        Args:
            query_vector: Pre-generated query embedding vector
            top_k: Number of results to return (default: 5)
            score_threshold: Minimum similarity score (default: 0.50)

        Returns:
            List of retrieval results sorted by relevance
        """
        if not query_vector:
            logger.warning("empty_query_vector_full_book")
            return []

        # Search vector store with full_book filter
        results = await self.vector_store.search(
            query_vector=query_vector,
            top_k=top_k,
            score_threshold=score_threshold,
            filter_conditions={"query_mode": "full_book"},
        )

        # Convert to RetrievalResult objects
        retrieval_results = []
        for result in results:
            payload = result["payload"]
            retrieval_results.append(
                RetrievalResult(
                    text=payload.get("text", ""),
                    score=result["score"],
                    chapter_title=payload.get("chapter_title", "Unknown"),
                    module=payload.get("module", "Unknown"),
                    source_url=payload.get("source_url", ""),
                    chunk_order=payload.get("chunk_order", 0),
                )
            )

        logger.info(
            "full_book_retrieval_complete",
            vector_size=len(query_vector),
            results_count=len(retrieval_results),
            top_score=retrieval_results[0].score if retrieval_results else 0.0,
        )

        return retrieval_results

    async def retrieve_selected_text(
        self,
        query_vector: list[float],
        session_id: UUID,
        top_k: int = 5,
        score_threshold: float = 0.40,
    ) -> list[RetrievalResult]:
        """
        Retrieve relevant chunks from selected text session.

        Args:
            query_vector: Pre-generated query embedding vector
            session_id: Session UUID for selected text
            top_k: Number of results to return (default: 5)
            score_threshold: Minimum similarity score (default: 0.40, lower for smaller context)

        Returns:
            List of retrieval results from selected text only
        """
        if not query_vector:
            logger.warning("empty_query_vector_selected_text", session_id=str(session_id))
            return []

        # Search vector store with selected_text and session_id filters
        results = await self.vector_store.search(
            query_vector=query_vector,
            top_k=top_k,
            score_threshold=score_threshold,
            filter_conditions={
                "query_mode": "selected_text",
                "session_id": str(session_id),
                "is_selected": True,
            },
        )

        # Convert to RetrievalResult objects
        retrieval_results = []
        for result in results:
            payload = result["payload"]
            retrieval_results.append(
                RetrievalResult(
                    text=payload.get("text", ""),
                    score=result["score"],
                    chapter_title=payload.get("chapter_title", "Selected Text"),
                    module=payload.get("module", ""),
                    source_url=payload.get("source_url", ""),
                    chunk_order=payload.get("chunk_order", 0),
                )
            )

        logger.info(
            "selected_text_retrieval_complete",
            session_id=str(session_id),
            vector_size=len(query_vector),
            results_count=len(retrieval_results),
            top_score=retrieval_results[0].score if retrieval_results else 0.0,
        )

        return retrieval_results

    def format_context(self, results: list[RetrievalResult]) -> str:
        """
        Format retrieval results into context string for LLM.

        Args:
            results: List of retrieval results

        Returns:
            Formatted context string with chapter attributions
        """
        if not results:
            return ""

        context_parts = []
        for i, result in enumerate(results, 1):
            # Format: "From [Chapter Title]: [text]"
            context_part = f"From {result.chapter_title}:\n{result.text}"
            context_parts.append(context_part)

        context = "\n\n".join(context_parts)

        logger.debug(
            "context_formatted",
            chunks_count=len(results),
            context_length=len(context),
        )

        return context

    def extract_sources(
        self, results: list[RetrievalResult]
    ) -> list[dict[str, Any]]:
        """
        Extract source references from retrieval results.

        Args:
            results: List of retrieval results

        Returns:
            List of source reference dictionaries
        """
        sources = []
        for result in results:
            source = {
                "chapter_title": result.chapter_title,
                "module": result.module,
                "source_url": result.source_url,
                "relevance_score": result.score,
            }
            sources.append(source)

        logger.debug(
            "sources_extracted",
            sources_count=len(sources),
        )

        return sources
