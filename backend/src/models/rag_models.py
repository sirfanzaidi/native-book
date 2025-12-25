"""
Pydantic models for RAG API requests and responses.

This module defines request/response schemas for the RAG chatbot API
with validation rules and examples.
"""

from typing import Any, Optional
from uuid import UUID

from pydantic import BaseModel, Field, field_validator


class QueryRequest(BaseModel):
    """Request model for RAG query endpoint."""

    query: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User query text",
        examples=["What is ROS2?"],
    )
    mode: str = Field(
        default="full_book",
        description="Query mode: 'full_book' or 'selected_text'",
        examples=["full_book", "selected_text"],
    )
    session_id: Optional[UUID] = Field(
        default=None,
        description="Session ID (required for selected_text mode)",
        examples=["123e4567-e89b-12d3-a456-426614174000"],
    )

    @field_validator("mode")
    @classmethod
    def validate_mode(cls, v: str) -> str:
        """Validate query mode is one of allowed values."""
        allowed_modes = {"full_book", "selected_text"}
        if v not in allowed_modes:
            raise ValueError(
                f"mode must be one of {allowed_modes}, got '{v}'"
            )
        return v

    @field_validator("session_id")
    @classmethod
    def validate_session_for_selected_text(cls, v: Optional[UUID], info) -> Optional[UUID]:
        """Validate session_id is provided for selected_text mode."""
        # Access mode from values if available
        if hasattr(info, 'data') and info.data.get("mode") == "selected_text" and v is None:
            raise ValueError(
                "session_id is required when mode is 'selected_text'"
            )
        return v

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "query": "What is ROS2?",
                    "mode": "full_book",
                },
                {
                    "query": "Explain this code",
                    "mode": "selected_text",
                    "session_id": "123e4567-e89b-12d3-a456-426614174000",
                },
            ]
        }
    }


class SourceReference(BaseModel):
    """Source reference model."""

    chapter_title: str = Field(
        ...,
        description="Chapter or section title",
        examples=["Introduction to ROS2"],
    )
    module: str = Field(
        ...,
        description="Module identifier",
        examples=["01-module-1-ros2"],
    )
    source_url: str = Field(
        ...,
        description="URL to source document",
        examples=["https://yourusername.github.io/module-1-ros2/01-introduction"],
    )
    relevance_score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Relevance score (0-1)",
        examples=[0.87],
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "chapter_title": "Introduction to ROS2",
                    "module": "01-module-1-ros2",
                    "source_url": "https://yourusername.github.io/module-1-ros2/01-introduction",
                    "relevance_score": 0.87,
                }
            ]
        }
    }


class QueryResponse(BaseModel):
    """Response model for RAG query endpoint."""

    answer: str = Field(
        ...,
        description="Generated answer text",
        examples=["ROS2 is a robotics middleware framework..."],
    )
    sources: list[SourceReference] = Field(
        ...,
        description="Source references for the answer",
    )
    latency_ms: int = Field(
        ...,
        ge=0,
        description="Query processing latency in milliseconds",
        examples=[1250],
    )
    model_used: str = Field(
        default="anthropic/claude-3-haiku",
        description="LLM model used for generation",
        examples=["anthropic/claude-3-haiku"],
    )
    query_mode: str = Field(
        ...,
        description="Query mode used",
        examples=["full_book", "selected_text"],
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "answer": "ROS2 is a robotics middleware framework that provides tools and libraries for building robot applications.",
                    "sources": [
                        {
                            "chapter_title": "Introduction to ROS2",
                            "module": "01-module-1-ros2",
                            "source_url": "https://yourusername.github.io/module-1-ros2/01-introduction",
                            "relevance_score": 0.87,
                        }
                    ],
                    "latency_ms": 1250,
                    "model_used": "anthropic/claude-3-haiku",
                    "query_mode": "full_book",
                }
            ]
        }
    }


class SelectedTextSessionRequest(BaseModel):
    """Request model for creating selected text session."""

    selected_text: str = Field(
        ...,
        min_length=10,
        max_length=10000,
        description="User-selected text to index",
        examples=["This is a code snippet that demonstrates..."],
    )
    metadata: Optional[dict[str, Any]] = Field(
        default=None,
        description="Optional metadata for the selected text",
        examples=[{"source": "user_selection", "page": "module-1"}],
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "selected_text": "def main():\n    rclpy.init()\n    node = MyNode()\n    rclpy.spin(node)",
                    "metadata": {"source": "code_snippet"},
                }
            ]
        }
    }


class SelectedTextSessionResponse(BaseModel):
    """Response model for selected text session creation."""

    session_id: UUID = Field(
        ...,
        description="Session UUID for subsequent queries",
        examples=["123e4567-e89b-12d3-a456-426614174000"],
    )
    expires_at: str = Field(
        ...,
        description="Session expiry timestamp (ISO 8601)",
        examples=["2025-12-23T17:00:00Z"],
    )
    chunks_indexed: int = Field(
        ...,
        ge=0,
        description="Number of chunks created from selected text",
        examples=[3],
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "session_id": "123e4567-e89b-12d3-a456-426614174000",
                    "expires_at": "2025-12-23T17:00:00Z",
                    "chunks_indexed": 3,
                }
            ]
        }
    }


class ErrorResponse(BaseModel):
    """Error response model."""

    error: str = Field(
        ...,
        description="Error type",
        examples=["validation_error", "not_found", "rate_limit_exceeded"],
    )
    message: str = Field(
        ...,
        description="Human-readable error message",
        examples=["Query text is required"],
    )
    details: Optional[dict[str, Any]] = Field(
        default=None,
        description="Additional error details",
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "error": "validation_error",
                    "message": "Query text must be between 1 and 2000 characters",
                },
                {
                    "error": "rate_limit_exceeded",
                    "message": "Rate limit exceeded. Please try again later.",
                    "details": {"retry_after_seconds": 60},
                },
            ]
        }
    }
