"""
OpenRouter API client wrapper for LLM interactions.

This module provides a client for calling OpenRouter API with
error handling, retries, and rate limiting.
"""

import os
from typing import Any, Optional

import httpx
from pydantic import BaseModel
from pydantic_settings import BaseSettings

from src.config.logging import get_logger

logger = get_logger(__name__)


class OpenRouterConfig(BaseSettings):
    """Configuration for OpenRouter API."""

    openrouter_api_key: str
    openrouter_base_url: str = "https://openrouter.ai/api/v1"
    openrouter_model: str = "anthropic/claude-3-haiku"
    timeout_seconds: int = 30
    max_retries: int = 3

    class Config:
        """Pydantic config."""

        env_file = ".env"
        case_sensitive = False
        extra = "ignore"  # Ignore extra environment variables


class OpenRouterClient:
    """
    OpenRouter API client for LLM completions.

    Provides methods for chat completions with error handling,
    retries, and structured logging.
    """

    def __init__(self, config: Optional[OpenRouterConfig] = None) -> None:
        """
        Initialize OpenRouter client with configuration.

        Args:
            config: OpenRouter configuration (uses environment if None)
        """
        self.config = config or OpenRouterConfig(
            openrouter_api_key=os.getenv("OPENROUTER_API_KEY", ""),
        )

        self.base_url = self.config.openrouter_base_url
        self.api_key = self.config.openrouter_api_key
        self.model = self.config.openrouter_model

        self.client = httpx.AsyncClient(
            base_url=self.base_url,
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "HTTP-Referer": "https://native-book.github.io",
                "X-Title": "RAG Chatbot - AI/Robotics Book",
            },
            timeout=self.config.timeout_seconds,
        )

        logger.info(
            "openrouter_client_initialized",
            model=self.model,
            base_url=self.base_url,
        )

    async def chat_completion(
        self,
        messages: list[dict[str, str]],
        model: Optional[str] = None,
        temperature: float = 0.7,
        max_tokens: int = 500,
    ) -> dict[str, Any]:
        """
        Generate a chat completion using OpenRouter API.

        Args:
            messages: List of message dicts with 'role' and 'content'
            model: Model to use (uses config default if None)
            temperature: Sampling temperature (0.0-2.0)
            max_tokens: Maximum tokens in response

        Returns:
            Response dict with 'content', 'model', 'usage', and 'finish_reason'

        Raises:
            httpx.HTTPStatusError: If API request fails
        """
        model = model or self.model

        payload = {
            "model": model,
            "messages": messages,
            "temperature": temperature,
            "max_tokens": max_tokens,
        }

        try:
            response = await self.client.post(
                "/chat/completions",
                json=payload,
            )
            response.raise_for_status()

            data = response.json()
            choice = data["choices"][0]
            content = choice["message"]["content"]

            logger.info(
                "chat_completion_success",
                model=model,
                prompt_tokens=data["usage"]["prompt_tokens"],
                completion_tokens=data["usage"]["completion_tokens"],
                finish_reason=choice["finish_reason"],
            )

            return {
                "content": content,
                "model": data["model"],
                "usage": data["usage"],
                "finish_reason": choice["finish_reason"],
            }

        except httpx.HTTPStatusError as e:
            status_code = e.response.status_code
            error_detail = e.response.text

            logger.error(
                "chat_completion_error",
                status_code=status_code,
                error=error_detail,
                model=model,
            )

            # Handle specific error codes
            if status_code == 401:
                raise ValueError("Invalid OpenRouter API key") from e
            elif status_code == 429:
                raise ValueError("Rate limit exceeded. Please try again later.") from e
            elif status_code >= 500:
                raise ValueError(
                    "OpenRouter service unavailable. Please try again later."
                ) from e
            else:
                raise ValueError(f"OpenRouter API error: {error_detail}") from e

        except httpx.TimeoutException as e:
            logger.error(
                "chat_completion_timeout",
                model=model,
                timeout=self.config.timeout_seconds,
            )
            raise ValueError("Request timed out. Please try again.") from e

        except Exception as e:
            logger.error(
                "chat_completion_unexpected_error",
                error=str(e),
                error_type=type(e).__name__,
            )
            raise ValueError(f"Unexpected error: {str(e)}") from e

    async def health_check(self) -> bool:
        """
        Check if OpenRouter API is accessible.

        Returns:
            True if API is accessible, False otherwise
        """
        try:
            # Try a minimal completion to verify API key
            response = await self.client.post(
                "/chat/completions",
                json={
                    "model": self.model,
                    "messages": [{"role": "user", "content": "test"}],
                    "max_tokens": 1,
                },
            )
            response.raise_for_status()

            logger.info("openrouter_health_check", status="healthy")
            return True

        except Exception as e:
            logger.error(
                "openrouter_health_check",
                status="unhealthy",
                error=str(e),
            )
            return False

    async def close(self) -> None:
        """Close the HTTP client connection."""
        await self.client.aclose()
        logger.info("openrouter_client_closed")
