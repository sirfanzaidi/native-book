"""
Book content indexing service for RAG system.

This module provides functionality to index markdown documents
into the vector store with embeddings and metadata.
"""

import os
from pathlib import Path
from typing import Optional
from uuid import uuid4

from qdrant_client.models import PointStruct

from src.config.logging import get_logger
from src.core.vector_store import QdrantVectorStore
from src.services.chunker import TextChunker
from src.services.embeddings import EmbeddingService
from src.services.metadata_extractor import MetadataExtractor

logger = get_logger(__name__)


class BookContentIndexer:
    """
    Service for indexing book content into vector store.

    Orchestrates chunking, embedding, and vector storage for
    markdown documents.
    """

    def __init__(
        self,
        vector_store: QdrantVectorStore,
        embedding_service: EmbeddingService,
        chunker: Optional[TextChunker] = None,
        metadata_extractor: Optional[MetadataExtractor] = None,
    ) -> None:
        """
        Initialize book content indexer.

        Args:
            vector_store: Qdrant vector store instance
            embedding_service: Embedding service instance
            chunker: Text chunker (creates default if None)
            metadata_extractor: Metadata extractor (creates default if None)
        """
        self.vector_store = vector_store
        self.embedding_service = embedding_service
        self.chunker = chunker or TextChunker()
        self.metadata_extractor = metadata_extractor or MetadataExtractor()

        logger.info("book_content_indexer_initialized")

    async def index_file(
        self,
        file_path: str,
        query_mode: str = "full_book",
    ) -> int:
        """
        Index a single markdown file into vector store.

        Args:
            file_path: Path to markdown file
            query_mode: Query mode for chunks ('full_book' or 'selected_text')

        Returns:
            Number of chunks indexed
        """
        try:
            # Read file content
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()

            # Extract metadata
            file_metadata = self.metadata_extractor.extract_file_metadata(
                file_path, content
            )

            # Remove frontmatter from content for chunking
            content_no_frontmatter = self._remove_frontmatter(content)

            # Chunk the content
            chunks = self.chunker.chunk_markdown(
                content_no_frontmatter,
                metadata={
                    "chapter_title": file_metadata["chapter_title"],
                    "module": file_metadata["module"],
                    "source_url": file_metadata["source_url"],
                    "file_path": file_metadata["file_path"],
                },
            )

            if not chunks:
                logger.warning(
                    "no_chunks_created",
                    file_path=file_path,
                )
                return 0

            # Generate embeddings for all chunks
            chunk_texts = [chunk["text"] for chunk in chunks]
            embeddings = self.embedding_service.embed_batch(chunk_texts)

            # Create Qdrant points
            points = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                point_id = str(uuid4())

                payload = {
                    "text": chunk["text"],
                    "chapter_title": chunk.get("chapter_title", ""),
                    "module": chunk.get("module", ""),
                    "source_url": chunk.get("source_url", ""),
                    "file_path": chunk.get("file_path", ""),
                    "chunk_order": chunk.get("chunk_order", i),
                    "word_count": chunk.get("word_count", 0),
                    "query_mode": query_mode,
                }

                point = PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload=payload,
                )
                points.append(point)

            # Upsert to vector store
            await self.vector_store.upsert_vectors(points)

            logger.info(
                "file_indexed",
                file_path=file_path,
                chunks_count=len(chunks),
                query_mode=query_mode,
            )

            return len(chunks)

        except Exception as e:
            logger.error(
                "file_indexing_error",
                file_path=file_path,
                error=str(e),
                error_type=type(e).__name__,
            )
            raise

    async def index_directory(
        self,
        directory_path: str,
        query_mode: str = "full_book",
        pattern: str = "**/*.md",
    ) -> dict[str, int]:
        """
        Index all markdown files in a directory.

        Args:
            directory_path: Path to directory containing markdown files
            query_mode: Query mode for chunks ('full_book' or 'selected_text')
            pattern: Glob pattern for matching files (default: "**/*.md")

        Returns:
            Dictionary with indexing statistics
        """
        directory = Path(directory_path)

        if not directory.exists():
            raise ValueError(f"Directory not found: {directory_path}")

        # Find all markdown files
        markdown_files = list(directory.glob(pattern))

        if not markdown_files:
            logger.warning(
                "no_markdown_files_found",
                directory=directory_path,
                pattern=pattern,
            )
            return {
                "files_processed": 0,
                "files_failed": 0,
                "total_chunks": 0,
            }

        logger.info(
            "indexing_directory",
            directory=directory_path,
            files_found=len(markdown_files),
            pattern=pattern,
        )

        # Index each file
        files_processed = 0
        files_failed = 0
        total_chunks = 0

        for file_path in markdown_files:
            try:
                chunks_count = await self.index_file(str(file_path), query_mode)
                total_chunks += chunks_count
                files_processed += 1
            except Exception as e:
                logger.error(
                    "file_indexing_failed",
                    file_path=str(file_path),
                    error=str(e),
                )
                files_failed += 1

        stats = {
            "files_processed": files_processed,
            "files_failed": files_failed,
            "total_chunks": total_chunks,
        }

        logger.info(
            "directory_indexing_complete",
            **stats,
        )

        return stats

    async def index_selected_text(
        self,
        text: str,
        session_id: str,
        metadata: Optional[dict] = None,
    ) -> int:
        """
        Index user-selected text for session-based queries.

        Args:
            text: Selected text to index
            session_id: Session identifier
            metadata: Optional metadata to attach

        Returns:
            Number of chunks indexed
        """
        # Chunk the selected text
        chunks = self.chunker.chunk_text(
            text,
            metadata=metadata or {},
        )

        if not chunks:
            logger.warning(
                "no_chunks_from_selected_text",
                session_id=session_id,
            )
            return 0

        # Generate embeddings
        chunk_texts = [chunk["text"] for chunk in chunks]
        embeddings = self.embedding_service.embed_batch(chunk_texts)

        # Create Qdrant points with session metadata
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            point_id = str(uuid4())

            payload = {
                "text": chunk["text"],
                "chunk_order": chunk.get("chunk_order", i),
                "word_count": chunk.get("word_count", 0),
                "query_mode": "selected_text",
                "session_id": session_id,
                "is_selected": True,
            }

            # Add any additional metadata
            if metadata:
                payload.update(metadata)

            point = PointStruct(
                id=point_id,
                vector=embedding,
                payload=payload,
            )
            points.append(point)

        # Upsert to vector store
        await self.vector_store.upsert_vectors(points)

        logger.info(
            "selected_text_indexed",
            session_id=session_id,
            chunks_count=len(chunks),
        )

        return len(chunks)

    def _remove_frontmatter(self, content: str) -> str:
        """
        Remove YAML frontmatter from markdown content.

        Args:
            content: Raw markdown content

        Returns:
            Content without frontmatter
        """
        import re

        frontmatter_pattern = re.compile(r"^---\s*\n.*?\n---\s*\n", re.DOTALL)
        return frontmatter_pattern.sub("", content, count=1)
