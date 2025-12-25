"""
Qdrant vector store client wrapper for RAG chatbot.

This module provides a configured Qdrant client with methods for
creating collections, upserting vectors, and searching for similar chunks.
"""

import os
from typing import Any, Optional

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    HnswConfigDiff,
    PointStruct,
    VectorParams,
    Filter,
    FieldCondition,
    MatchValue,
    SearchRequest,
    PayloadSchemaType,
)
from pydantic import BaseModel
from pydantic_settings import BaseSettings

from src.config.logging import get_logger

logger = get_logger(__name__)


class VectorStoreConfig(BaseSettings):
    """Configuration for Qdrant vector store."""

    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "book_embeddings"
    qdrant_vector_size: int = 384
    qdrant_distance: str = "Cosine"

    class Config:
        """Pydantic config."""

        env_file = ".env"
        case_sensitive = False
        extra = "ignore"  # Ignore extra environment variables


class QdrantVectorStore:
    """
    Qdrant vector database client wrapper.

    Provides methods for creating collections, inserting vectors,
    and performing similarity search with metadata filtering.
    """

    def __init__(self, config: Optional[VectorStoreConfig] = None) -> None:
        """
        Initialize Qdrant client with configuration.

        Args:
            config: Vector store configuration (uses environment if None)
        """
        self.config = config or VectorStoreConfig(
            qdrant_url=os.getenv("QDRANT_URL", ""),
            qdrant_api_key=os.getenv("QDRANT_API_KEY", ""),
        )

        self.client = QdrantClient(
            url=self.config.qdrant_url,
            api_key=self.config.qdrant_api_key,
        )

        self.collection_name = self.config.qdrant_collection_name
        self.vector_size = self.config.qdrant_vector_size

        logger.info(
            "vector_store_initialized",
            collection=self.collection_name,
            vector_size=self.vector_size,
        )

    async def create_collection(
        self,
        collection_name: Optional[str] = None,
        vector_size: Optional[int] = None,
        distance: Distance = Distance.COSINE,
        hnsw_m: int = 12,
        hnsw_ef_construct: int = 150,
    ) -> None:
        """
        Create a Qdrant collection with HNSW indexing.

        Args:
            collection_name: Collection name (uses config default if None)
            vector_size: Vector dimension (uses config default if None)
            distance: Distance metric (Cosine, Dot, Euclidean)
            hnsw_m: HNSW m parameter (number of edges per node)
            hnsw_ef_construct: HNSW ef_construct parameter (search depth during construction)
        """
        collection_name = collection_name or self.collection_name
        vector_size = vector_size or self.vector_size

        # Check if collection already exists
        collections = self.client.get_collections().collections
        if any(c.name == collection_name for c in collections):
            logger.info(
                "collection_exists",
                collection=collection_name,
                message="Collection already exists, ensuring indexes are created",
            )
            # Ensure payload indexes exist even for existing collections
            await self._create_payload_indexes(collection_name)
            return

        self.client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=vector_size,
                distance=distance,
                hnsw_config=HnswConfigDiff(
                    m=hnsw_m,
                    ef_construct=hnsw_ef_construct,
                ),
            ),
        )

        logger.info(
            "collection_created",
            collection=collection_name,
            vector_size=vector_size,
            distance=distance.value,
            hnsw_m=hnsw_m,
            hnsw_ef_construct=hnsw_ef_construct,
        )

        # Create payload indexes for filtering
        await self._create_payload_indexes(collection_name)

    async def _create_payload_indexes(self, collection_name: str) -> None:
        """
        Create payload field indexes for efficient filtering.

        Args:
            collection_name: Collection name to create indexes for
        """
        index_fields = ["query_mode", "session_id", "is_selected"]

        for field_name in index_fields:
            try:
                self.client.create_payload_index(
                    collection_name=collection_name,
                    field_name=field_name,
                    field_schema=PayloadSchemaType.KEYWORD,
                )
                logger.debug(
                    "payload_index_created",
                    collection=collection_name,
                    field=field_name,
                )
            except Exception as e:
                # Index may already exist, which is fine
                logger.debug(
                    "payload_index_creation_skipped",
                    collection=collection_name,
                    field=field_name,
                    reason=str(e),
                )

        logger.info(
            "payload_indexes_ensured",
            collection=collection_name,
            fields=index_fields,
        )

    async def upsert_vectors(
        self,
        points: list[PointStruct],
        collection_name: Optional[str] = None,
    ) -> None:
        """
        Insert or update vectors in the collection.

        Args:
            points: List of PointStruct objects with id, vector, and payload
            collection_name: Collection name (uses config default if None)
        """
        collection_name = collection_name or self.collection_name

        self.client.upsert(
            collection_name=collection_name,
            points=points,
        )

        logger.info(
            "vectors_upserted",
            collection=collection_name,
            count=len(points),
        )

    async def search(
        self,
        query_vector: list[float],
        top_k: int = 5,
        score_threshold: float = 0.5,
        filter_conditions: Optional[dict[str, Any]] = None,
        collection_name: Optional[str] = None,
    ) -> list[dict[str, Any]]:
        """
        Search for similar vectors with optional metadata filtering.

        Args:
            query_vector: Query embedding vector
            top_k: Number of results to return
            score_threshold: Minimum similarity score (0-1)
            filter_conditions: Metadata filter conditions (e.g., {"query_mode": "full_book"})
            collection_name: Collection name (uses config default if None)

        Returns:
            List of search results with payload and score
        """
        collection_name = collection_name or self.collection_name

        # Build filter if conditions provided
        search_filter = None
        if filter_conditions:
            must_conditions = [
                FieldCondition(
                    key=key,
                    match=MatchValue(value=value),
                )
                for key, value in filter_conditions.items()
            ]
            search_filter = Filter(must=must_conditions)

        results = self.client.query_points(
            collection_name=collection_name,
            query=query_vector,
            limit=top_k,
            score_threshold=score_threshold,
            query_filter=search_filter,
        ).points

        logger.info(
            "vector_search_completed",
            collection=collection_name,
            top_k=top_k,
            score_threshold=score_threshold,
            results_count=len(results),
            filter=filter_conditions,
        )

        return [
            {
                "id": result.id,
                "score": result.score,
                "payload": result.payload,
            }
            for result in results
        ]

    async def delete_by_filter(
        self,
        filter_conditions: dict[str, Any],
        collection_name: Optional[str] = None,
    ) -> None:
        """
        Delete vectors matching filter conditions.

        Args:
            filter_conditions: Metadata filter conditions
            collection_name: Collection name (uses config default if None)
        """
        collection_name = collection_name or self.collection_name

        must_conditions = [
            FieldCondition(
                key=key,
                match=MatchValue(value=value),
            )
            for key, value in filter_conditions.items()
        ]
        delete_filter = Filter(must=must_conditions)

        self.client.delete(
            collection_name=collection_name,
            points_selector=delete_filter,
        )

        logger.info(
            "vectors_deleted",
            collection=collection_name,
            filter=filter_conditions,
        )

    async def health_check(self) -> bool:
        """
        Check if Qdrant service is healthy.

        Returns:
            True if service is accessible, False otherwise
        """
        try:
            self.client.get_collections()
            logger.info("qdrant_health_check", status="healthy")
            return True
        except Exception as e:
            logger.error("qdrant_health_check", status="unhealthy", error=str(e))
            return False
