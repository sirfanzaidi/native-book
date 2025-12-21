# Contributing to the Physical AI & Humanoid Robotics Book

Thank you for your interest in contributing! This document outlines how to contribute chapters, code examples, and improvements.

## Chapter Writing Process

### 1. Content Structure

Each chapter must follow this structure:

```markdown
---
id: chapter-id
title: "Chapter Title"
sidebar_label: "Short Title"
---

# Chapter Title

## Introduction
- Brief overview (2-3 paragraphs)
- Why this topic matters
- What readers will learn

## Core Concepts
- Explain fundamental ideas
- Use visual descriptions (ASCII diagrams welcome)
- Reference official documentation

## Code Examples
- Start simple, progress to complex
- Use "setup → code → output → explanation" pattern
- Include expected output
- Test all examples in clean environment

## Summary
- Key takeaways (bullet list)
- Next steps
- References (APA format)
```

### 2. Code Examples

All code examples must:
- ✅ Run without modification
- ✅ Include exact dependency versions
- ✅ Have inline comments explaining key lines
- ✅ Show expected output
- ✅ Be tested on at least 2 platforms (Ubuntu, macOS, or Windows WSL2)

### 3. Citations

All claims must be traceable:
- Link to official documentation (ROS 2, Gazebo, NVIDIA Isaac, OpenAI)
- Use APA 7th edition format
- Include access date: (Accessed: YYYY-MM-DD)

### 4. Code Example Testing

Before submitting:

```bash
# Install dependencies listed in chapter
python3 example_code.py

# Verify output matches documented output
```

## File Organization

```
docs/
├── 01-module-1-ros2/
│   ├── 01-introduction.md
│   ├── 02-services.md         (future)
│   └── examples/
│       ├── simple_publisher.py
│       ├── simple_subscriber.py
│       └── README.md
├── 02-module-2-digital-twin/  (coming soon)
└── ...
```

## Pull Request Process

1. Fork the repository
2. Create a feature branch: `git checkout -b chapter/my-chapter`
3. Write content following guidelines above
4. Test all examples
5. Verify all citations
6. Submit PR with description of changes

## Review Criteria

PRs will be reviewed for:
- ✅ Technical accuracy (verified against official docs)
- ✅ Code reproducibility (tested on clean environment)
- ✅ Clarity and readability
- ✅ Proper citations
- ✅ Markdown formatting

## Questions?

Open an issue or start a discussion in the repository.

Thank you for contributing! 📚
