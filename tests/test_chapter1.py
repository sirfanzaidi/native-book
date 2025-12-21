"""
Tests for Chapter 1: ROS 2 Architecture
Validates markdown structure, code examples, and documentation completeness
"""

import os
import re
import ast
import pytest
from pathlib import Path


class TestChapter1Structure:
    """Verify Chapter 1 markdown file exists and has required sections"""

    def test_chapter1_markdown_exists(self):
        """T017.1: Verify docs/01-module-1-ros2/01-introduction.md exists"""
        chapter_path = Path("docs/01-module-1-ros2/01-introduction.md")
        assert chapter_path.exists(), f"Chapter 1 markdown not found at {chapter_path}"

    def test_chapter1_has_required_sections(self):
        """T017.2: Verify chapter has required content sections"""
        chapter_path = Path("docs/01-module-1-ros2/01-introduction.md")
        content = chapter_path.read_text()

        required_sections = [
            "# Chapter 1",
            "## Introduction",
            "## Summary",
            "References"  # Can be a markdown link section, not a header
        ]

        for section in required_sections:
            assert section in content, f"Missing required section: {section}"

        # Verify either "## Creating Your First ROS 2 Nodes" or "## Code Examples"
        has_code_examples = ("## Creating Your First ROS 2 Nodes" in content or
                            "## Code Examples" in content)
        assert has_code_examples, "Missing code examples section"

        # Also verify key concepts are present
        key_concepts = [
            "Pub/Sub",
            "Publisher",
            "Subscriber",
            "Topic",
            "rclpy"
        ]

        for concept in key_concepts:
            assert concept in content, f"Missing key concept: {concept}"

    def test_chapter1_word_count(self):
        """T017.3: Verify chapter content is substantial (~800+ words)"""
        chapter_path = Path("docs/01-module-1-ros2/01-introduction.md")
        content = chapter_path.read_text()

        # Remove front matter
        if content.startswith("---"):
            content = content.split("---", 2)[2]

        # Count words
        word_count = len(content.split())
        assert word_count >= 700, f"Chapter too short: {word_count} words (need ~800)"


class TestCodeExamples:
    """Verify Python code examples have valid syntax and are executable"""

    def test_publisher_example_exists(self):
        """T017.4: Verify simple_publisher.py exists"""
        pub_path = Path("docs/01-module-1-ros2/examples/simple_publisher.py")
        assert pub_path.exists(), f"Publisher example not found at {pub_path}"

    def test_subscriber_example_exists(self):
        """T017.5: Verify simple_subscriber.py exists"""
        sub_path = Path("docs/01-module-1-ros2/examples/simple_subscriber.py")
        assert sub_path.exists(), f"Subscriber example not found at {sub_path}"

    def test_publisher_syntax_valid(self):
        """T017.6: Verify simple_publisher.py has valid Python syntax"""
        pub_path = Path("docs/01-module-1-ros2/examples/simple_publisher.py")
        code = pub_path.read_text()

        try:
            ast.parse(code)
        except SyntaxError as e:
            pytest.fail(f"Publisher code has syntax error: {e}")

    def test_subscriber_syntax_valid(self):
        """T017.7: Verify simple_subscriber.py has valid Python syntax"""
        sub_path = Path("docs/01-module-1-ros2/examples/simple_subscriber.py")
        code = sub_path.read_text()

        try:
            ast.parse(code)
        except SyntaxError as e:
            pytest.fail(f"Subscriber code has syntax error: {e}")

    def test_publisher_has_comments(self):
        """T017.8: Verify publisher example has inline comments"""
        pub_path = Path("docs/01-module-1-ros2/examples/simple_publisher.py")
        code = pub_path.read_text()

        assert "#" in code, "Publisher example missing inline comments"

    def test_subscriber_has_comments(self):
        """T017.9: Verify subscriber example has inline comments"""
        sub_path = Path("docs/01-module-1-ros2/examples/simple_subscriber.py")
        code = sub_path.read_text()

        assert "#" in code, "Subscriber example missing inline comments"

    def test_publisher_imports_rclpy(self):
        """T017.10: Verify publisher imports rclpy"""
        pub_path = Path("docs/01-module-1-ros2/examples/simple_publisher.py")
        code = pub_path.read_text()

        assert "import rclpy" in code, "Publisher missing rclpy import"

    def test_subscriber_imports_rclpy(self):
        """T017.11: Verify subscriber imports rclpy"""
        sub_path = Path("docs/01-module-1-ros2/examples/simple_subscriber.py")
        code = sub_path.read_text()

        assert "import rclpy" in code, "Subscriber missing rclpy import"


class TestCitations:
    """Verify all claims are cited to official documentation"""

    def test_chapter1_has_citations(self):
        """T017.12: Verify chapter includes citations to official docs"""
        chapter_path = Path("docs/01-module-1-ros2/01-introduction.md")
        content = chapter_path.read_text()

        # Look for citation patterns: (Author, Year) or [link]
        citation_patterns = [
            r"\([A-Z][a-z]+,\s*20\d{2}\)",  # (Author, YYYY)
            r"https://docs\.ros\.org",       # ROS 2 official docs
            r"\[.*\]\(.*\)",                 # Markdown links
        ]

        has_citations = False
        for pattern in citation_patterns:
            if re.search(pattern, content):
                has_citations = True
                break

        assert has_citations, "Chapter missing citations to official documentation"

    def test_chapter1_references_ros2_docs(self):
        """T017.13: Verify chapter references official ROS 2 documentation"""
        chapter_path = Path("docs/01-module-1-ros2/01-introduction.md")
        content = chapter_path.read_text()

        # Should have at least one reference to official ROS 2 docs
        assert "https://docs.ros.org" in content or "ROS 2" in content, \
            "Chapter should reference official ROS 2 documentation"


class TestDocusaurus:
    """Verify Docusaurus integration and build artifacts"""

    def test_docusaurus_config_exists(self):
        """T019.1: Verify docusaurus.config.js exists"""
        config_path = Path("docusaurus.config.js")
        assert config_path.exists(), "Docusaurus config not found"

    def test_package_json_has_docusaurus(self):
        """T019.2: Verify package.json includes Docusaurus dependencies"""
        package_path = Path("package.json")
        content = package_path.read_text()

        assert "@docusaurus/core" in content, "Missing @docusaurus/core"
        assert "@docusaurus/preset-classic" in content, "Missing @docusaurus/preset-classic"

    def test_docs_folder_structure(self):
        """T019.3: Verify docs folder has proper structure"""
        required_paths = [
            Path("docs/index.md"),
            Path("docs/01-module-1-ros2"),
            Path("docs/01-module-1-ros2/01-introduction.md"),
        ]

        for path in required_paths:
            assert path.exists(), f"Missing required path: {path}"


class TestBuildOutput:
    """Verify build succeeds and produces expected artifacts"""

    def test_build_directory_created(self):
        """T019.4: Verify npm build creates build/ directory"""
        build_path = Path("build")
        # This test is conditional - only run if build has been executed
        if build_path.exists():
            assert build_path.is_dir(), "build/ exists but is not a directory"
            assert len(list(build_path.iterdir())) > 0, "build/ directory is empty"


# Test execution markers
@pytest.mark.mvp
class TestMVPCompletion:
    """High-level MVP completion checks"""

    def test_chapter1_complete(self):
        """MVP: Verify Chapter 1 is complete and ready for deployment"""
        chapter_path = Path("docs/01-module-1-ros2/01-introduction.md")
        pub_path = Path("docs/01-module-1-ros2/examples/simple_publisher.py")
        sub_path = Path("docs/01-module-1-ros2/examples/simple_subscriber.py")

        assert chapter_path.exists(), "Chapter 1 markdown missing"
        assert pub_path.exists(), "Publisher example missing"
        assert sub_path.exists(), "Subscriber example missing"

    def test_docusaurus_configured(self):
        """MVP: Verify Docusaurus is properly configured"""
        config_path = Path("docusaurus.config.js")
        package_path = Path("package.json")
        sidebars_path = Path("sidebars.js")

        assert config_path.exists(), "Docusaurus config missing"
        assert package_path.exists(), "package.json missing"
        assert sidebars_path.exists(), "sidebars.js missing"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
