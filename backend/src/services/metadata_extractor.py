"""
Metadata extraction utility for markdown files.

This module provides functionality to extract structured metadata
from Docusaurus markdown files including frontmatter and content analysis.
"""

import os
import re
from pathlib import Path
from typing import Any, Optional

import yaml

from src.config.logging import get_logger

logger = get_logger(__name__)


class MetadataExtractor:
    """
    Markdown metadata extraction service.

    Extracts frontmatter, headings, and generates source URLs
    for Docusaurus book chapters.
    """

    def __init__(self, base_url: Optional[str] = None) -> None:
        """
        Initialize metadata extractor.

        Args:
            base_url: Base URL for generating source links (e.g., "https://yourusername.github.io")
        """
        self.base_url = base_url or os.getenv(
            "DOCS_BASE_URL", "https://yourusername.github.io"
        )

        logger.info(
            "metadata_extractor_initialized",
            base_url=self.base_url,
        )

    def extract_frontmatter(self, markdown_content: str) -> dict[str, Any]:
        """
        Extract YAML frontmatter from markdown content.

        Args:
            markdown_content: Raw markdown text

        Returns:
            Dictionary of frontmatter fields
        """
        frontmatter_pattern = re.compile(r"^---\s*\n(.*?)\n---\s*\n", re.DOTALL)
        match = frontmatter_pattern.match(markdown_content)

        if match:
            frontmatter_text = match.group(1)
            try:
                frontmatter = yaml.safe_load(frontmatter_text)
                logger.debug(
                    "frontmatter_extracted",
                    fields=list(frontmatter.keys()) if frontmatter else [],
                )
                return frontmatter or {}
            except yaml.YAMLError as e:
                logger.error(
                    "frontmatter_parse_error",
                    error=str(e),
                )
                return {}

        return {}

    def extract_headings(self, markdown_content: str) -> list[dict[str, str]]:
        """
        Extract markdown headings (h1-h6).

        Args:
            markdown_content: Raw markdown text

        Returns:
            List of heading dictionaries with 'level', 'text', and 'slug'
        """
        heading_pattern = re.compile(r"^(#{1,6})\s+(.+)$", re.MULTILINE)
        headings = []

        for match in heading_pattern.finditer(markdown_content):
            level = len(match.group(1))
            text = match.group(2).strip()
            slug = self._generate_slug(text)

            headings.append({
                "level": level,
                "text": text,
                "slug": slug,
            })

        logger.debug(
            "headings_extracted",
            count=len(headings),
        )

        return headings

    def generate_source_url(
        self,
        file_path: str,
        docs_dir: str = "docs",
    ) -> str:
        """
        Generate Docusaurus URL from file path.

        Args:
            file_path: Absolute or relative path to markdown file
            docs_dir: Documentation root directory name (default: "docs")

        Returns:
            Full URL to the document page
        """
        # Convert to Path object for easier manipulation
        path = Path(file_path)

        # Find the docs directory in the path
        parts = path.parts
        try:
            docs_index = parts.index(docs_dir)
            # Get path after docs/ directory
            relative_parts = parts[docs_index + 1 :]

            # Remove .md extension and convert to URL path
            url_parts = []
            for part in relative_parts:
                if part.endswith(".md"):
                    part = part[:-3]
                url_parts.append(part)

            # Join with base URL
            url_path = "/".join(url_parts)
            source_url = f"{self.base_url}/{url_path}"

            logger.debug(
                "source_url_generated",
                file_path=file_path,
                source_url=source_url,
            )

            return source_url

        except ValueError:
            # docs_dir not in path, return base URL
            logger.warning(
                "docs_dir_not_found_in_path",
                file_path=file_path,
                docs_dir=docs_dir,
            )
            return self.base_url

    def extract_file_metadata(
        self,
        file_path: str,
        content: Optional[str] = None,
    ) -> dict[str, Any]:
        """
        Extract comprehensive metadata from a markdown file.

        Args:
            file_path: Path to markdown file
            content: Optional file content (reads from disk if None)

        Returns:
            Dictionary with frontmatter, headings, source_url, and file info
        """
        # Read content if not provided
        if content is None:
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    content = f.read()
            except Exception as e:
                logger.error(
                    "file_read_error",
                    file_path=file_path,
                    error=str(e),
                )
                return {}

        # Extract components
        frontmatter = self.extract_frontmatter(content)
        headings = self.extract_headings(content)
        source_url = self.generate_source_url(file_path)

        # Get file info
        path = Path(file_path)
        file_name = path.name
        parent_dir = path.parent.name

        # Determine module and chapter from path
        # Example: docs/01-module-1-ros2/01-introduction.md
        module = self._extract_module_from_path(file_path)
        chapter_title = frontmatter.get("title") or self._extract_title_from_filename(
            file_name
        )

        metadata = {
            "file_path": file_path,
            "file_name": file_name,
            "parent_dir": parent_dir,
            "module": module,
            "chapter_title": chapter_title,
            "source_url": source_url,
            "frontmatter": frontmatter,
            "headings": headings,
            "word_count": len(content.split()),
        }

        logger.info(
            "file_metadata_extracted",
            file_path=file_path,
            module=module,
            chapter_title=chapter_title,
        )

        return metadata

    def _generate_slug(self, text: str) -> str:
        """
        Generate URL-safe slug from heading text.

        Args:
            text: Heading text

        Returns:
            URL-safe slug
        """
        # Convert to lowercase and replace spaces with hyphens
        slug = text.lower()
        slug = re.sub(r"[^\w\s-]", "", slug)
        slug = re.sub(r"[-\s]+", "-", slug)
        slug = slug.strip("-")

        return slug

    def _extract_module_from_path(self, file_path: str) -> str:
        """
        Extract module identifier from file path.

        Args:
            file_path: File path

        Returns:
            Module identifier (e.g., "module-1-ros2")
        """
        # Look for pattern like "01-module-1-ros2" in path
        module_pattern = re.compile(r"\d+-module-\d+-[\w-]+")
        match = module_pattern.search(file_path)

        if match:
            return match.group(0)

        # Fallback to parent directory name
        path = Path(file_path)
        return path.parent.name

    def _extract_title_from_filename(self, file_name: str) -> str:
        """
        Extract title from filename.

        Args:
            file_name: File name (e.g., "01-introduction.md")

        Returns:
            Title (e.g., "Introduction")
        """
        # Remove .md extension
        name = file_name[:-3] if file_name.endswith(".md") else file_name

        # Remove leading numbers and hyphens
        title = re.sub(r"^\d+-", "", name)

        # Replace hyphens with spaces and capitalize
        title = title.replace("-", " ").title()

        return title
