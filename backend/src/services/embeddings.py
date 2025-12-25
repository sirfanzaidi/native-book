"""
Embedding service using sentence-transformers.

This module provides text embedding functionality using the
all-MiniLM-L6-v2 model (384-dimensional vectors).
"""

import os
from typing import Optional

from sentence_transformers import SentenceTransformer

from src.config.logging import get_logger

logger = get_logger(__name__)


class EmbeddingService:
    """
    Text embedding service using sentence-transformers.

    Provides methods for generating 384-dimensional embeddings
    from text using the all-MiniLM-L6-v2 model.
    """

    def __init__(
        self,
        model_name: Optional[str] = None,
        device: Optional[str] = None,
    ) -> None:
        """
        Initialize embedding model.

        Args:
            model_name: HuggingFace model name (default: all-MiniLM-L6-v2)
            device: Device to use ('cpu', 'cuda', or None for auto-detect)
        """
        self.model_name = model_name or os.getenv(
            "EMBEDDING_MODEL_NAME",
            "sentence-transformers/all-MiniLM-L6-v2",
        )
        self.device = device or os.getenv("EMBEDDING_DEVICE", "cpu")

        logger.info(
            "loading_embedding_model",
            model=self.model_name,
            device=self.device,
        )

        self.model = SentenceTransformer(self.model_name, device=self.device)
        self.vector_size = self.model.get_sentence_embedding_dimension()

        logger.info(
            "embedding_model_loaded",
            model=self.model_name,
            vector_size=self.vector_size,
            device=self.device,
        )

    def embed_text(self, text: str) -> list[float]:
        """
        Generate embedding vector for a single text.

        Args:
            text: Input text to embed

        Returns:
            Embedding vector as list of floats (384 dimensions)
        """
        if not text or not text.strip():
            logger.warning("empty_text_embedding", text=text)
            # Return zero vector for empty text
            return [0.0] * self.vector_size

        embedding = self.model.encode(text, convert_to_numpy=True)

        logger.debug(
            "text_embedded",
            text_length=len(text),
            vector_size=len(embedding),
        )

        return embedding.tolist()

    def embed_batch(self, texts: list[str], batch_size: int = 32) -> list[list[float]]:
        """
        Generate embedding vectors for multiple texts efficiently.

        Args:
            texts: List of input texts to embed
            batch_size: Number of texts to process at once

        Returns:
            List of embedding vectors
        """
        if not texts:
            logger.warning("empty_batch_embedding")
            return []

        # Filter out empty texts and track indices
        non_empty_texts = []
        non_empty_indices = []
        for i, text in enumerate(texts):
            if text and text.strip():
                non_empty_texts.append(text)
                non_empty_indices.append(i)

        # Generate embeddings for non-empty texts
        if non_empty_texts:
            embeddings = self.model.encode(
                non_empty_texts,
                batch_size=batch_size,
                convert_to_numpy=True,
                show_progress_bar=False,
            )
            embeddings_list = embeddings.tolist()
        else:
            embeddings_list = []

        # Reconstruct full result list with zero vectors for empty texts
        result = []
        embedding_idx = 0
        for i in range(len(texts)):
            if i in non_empty_indices:
                result.append(embeddings_list[embedding_idx])
                embedding_idx += 1
            else:
                result.append([0.0] * self.vector_size)

        logger.info(
            "batch_embedded",
            batch_size=len(texts),
            non_empty_count=len(non_empty_texts),
            vector_size=self.vector_size,
        )

        return result

    def get_vector_size(self) -> int:
        """
        Get the dimensionality of embedding vectors.

        Returns:
            Vector dimension (384 for all-MiniLM-L6-v2)
        """
        return self.vector_size
