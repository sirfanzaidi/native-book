"""
Text chunking utility for splitting documents into smaller segments.

This module provides chunking functionality with configurable
token sizes and overlap for optimal RAG retrieval.
"""

import os
from typing import Optional

from src.config.logging import get_logger

logger = get_logger(__name__)


class TextChunker:
    """
    Text chunking service for splitting documents.

    Implements fixed-size chunking with overlap to maintain context
    across chunk boundaries.
    """

    def __init__(
        self,
        chunk_size_min: Optional[int] = None,
        chunk_size_max: Optional[int] = None,
        overlap_percent: Optional[int] = None,
    ) -> None:
        """
        Initialize text chunker with configuration.

        Args:
            chunk_size_min: Minimum chunk size in tokens (default: 512)
            chunk_size_max: Maximum chunk size in tokens (default: 1024)
            overlap_percent: Overlap percentage (default: 20)
        """
        self.chunk_size_min = chunk_size_min or int(
            os.getenv("CHUNK_SIZE_MIN", "512")
        )
        self.chunk_size_max = chunk_size_max or int(
            os.getenv("CHUNK_SIZE_MAX", "1024")
        )
        self.overlap_percent = overlap_percent or int(
            os.getenv("CHUNK_OVERLAP_PERCENT", "20")
        )

        # Calculate overlap size in tokens
        self.overlap_size = int(self.chunk_size_max * (self.overlap_percent / 100))

        logger.info(
            "text_chunker_initialized",
            chunk_size_min=self.chunk_size_min,
            chunk_size_max=self.chunk_size_max,
            overlap_percent=self.overlap_percent,
            overlap_size=self.overlap_size,
        )

    def chunk_text(self, text: str, metadata: Optional[dict] = None) -> list[dict]:
        """
        Split text into overlapping chunks.

        Args:
            text: Input text to chunk
            metadata: Optional metadata to attach to each chunk

        Returns:
            List of chunk dictionaries with 'text', 'chunk_order', and metadata
        """
        if not text or not text.strip():
            logger.warning("empty_text_chunking", text=text)
            return []

        # Simple word-based chunking (approximate token count)
        # In production, consider using tiktoken for accurate token counts
        words = text.split()
        chunks = []
        chunk_order = 0

        i = 0
        while i < len(words):
            # Determine chunk size (aim for max, but respect min)
            chunk_end = min(i + self.chunk_size_max, len(words))
            chunk_words = words[i:chunk_end]

            # Only create chunk if it meets minimum size or is the last chunk
            if len(chunk_words) >= self.chunk_size_min or chunk_end == len(words):
                chunk_text = " ".join(chunk_words)

                chunk_dict = {
                    "text": chunk_text,
                    "chunk_order": chunk_order,
                    "word_count": len(chunk_words),
                }

                # Add metadata if provided
                if metadata:
                    chunk_dict.update(metadata)

                chunks.append(chunk_dict)
                chunk_order += 1

            # Move forward with overlap
            i += self.chunk_size_max - self.overlap_size

            # Prevent infinite loop if overlap is too large
            if i <= chunks[-1]["word_count"] if chunks else 0:
                i += max(1, self.chunk_size_max // 2)

        logger.info(
            "text_chunked",
            total_words=len(words),
            chunks_created=len(chunks),
            avg_chunk_size=sum(c["word_count"] for c in chunks) // len(chunks)
            if chunks
            else 0,
        )

        return chunks

    def chunk_markdown(
        self, markdown_text: str, metadata: Optional[dict] = None
    ) -> list[dict]:
        """
        Split markdown text into chunks while preserving section boundaries.

        Args:
            markdown_text: Input markdown text
            metadata: Optional metadata to attach to each chunk

        Returns:
            List of chunk dictionaries
        """
        # For MVP, use simple text chunking
        # Future enhancement: respect markdown headings and code blocks
        return self.chunk_text(markdown_text, metadata)

    def chunk_by_sentences(
        self, text: str, metadata: Optional[dict] = None
    ) -> list[dict]:
        """
        Split text into chunks at sentence boundaries.

        Args:
            text: Input text to chunk
            metadata: Optional metadata to attach to each chunk

        Returns:
            List of chunk dictionaries
        """
        if not text or not text.strip():
            logger.warning("empty_text_sentence_chunking")
            return []

        # Simple sentence splitting (can be enhanced with NLTK or spaCy)
        import re

        sentences = re.split(r"(?<=[.!?])\s+", text)
        chunks = []
        chunk_order = 0

        current_chunk = []
        current_word_count = 0

        for sentence in sentences:
            sentence_words = sentence.split()
            sentence_word_count = len(sentence_words)

            # If adding this sentence exceeds max size, create chunk
            if (
                current_word_count + sentence_word_count > self.chunk_size_max
                and current_chunk
            ):
                chunk_text = " ".join(current_chunk)
                chunk_dict = {
                    "text": chunk_text,
                    "chunk_order": chunk_order,
                    "word_count": current_word_count,
                }

                if metadata:
                    chunk_dict.update(metadata)

                chunks.append(chunk_dict)
                chunk_order += 1

                # Keep overlap sentences
                overlap_word_count = int(current_word_count * (self.overlap_percent / 100))
                overlap_words = []
                for i in range(len(current_chunk) - 1, -1, -1):
                    overlap_words.insert(0, current_chunk[i])
                    if len(" ".join(overlap_words).split()) >= overlap_word_count:
                        break

                current_chunk = overlap_words
                current_word_count = len(" ".join(current_chunk).split())

            current_chunk.append(sentence)
            current_word_count += sentence_word_count

        # Add final chunk if exists
        if current_chunk and current_word_count >= self.chunk_size_min:
            chunk_text = " ".join(current_chunk)
            chunk_dict = {
                "text": chunk_text,
                "chunk_order": chunk_order,
                "word_count": current_word_count,
            }

            if metadata:
                chunk_dict.update(metadata)

            chunks.append(chunk_dict)

        logger.info(
            "text_sentence_chunked",
            total_sentences=len(sentences),
            chunks_created=len(chunks),
        )

        return chunks
