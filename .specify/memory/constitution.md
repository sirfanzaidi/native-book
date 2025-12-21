# AI/Spec-Driven Book on Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Technical Accuracy via Verification
All claims, code examples, and integrations MUST be traceable to official documentation or primary sources. When referencing external tools, APIs, or frameworks, verification through official docs and practical testing is mandatory before inclusion. Never assume; always verify against authoritative sources.

### II. Clarity for Target Audience
Content MUST be comprehensible to engineers and CS students with foundational knowledge of AI/robotics concepts. Code examples require inline explanation. Complex concepts broken into digestible sections. Avoid jargon without definition; provide context for all technical terms introduced.

### III. Full Reproducibility
All code samples, setup procedures, and integrations MUST be end-to-end reproducible by readers. Every command, configuration, and dependency MUST be documented with exact versions. No implicit assumptions; all prerequisites listed. Readers should run examples without modification and achieve identical results.

### IV. Rigor Through Established Tools & Frameworks
Use industry-standard, well-documented tools: Spec-Kit Plus, Claude Code, OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud, OpenRouter. Do not introduce experimental or undocumented tools. Preference for tools with active communities, clear documentation, and proven track records.

### V. Zero Plagiarism, Original Content
All content (text, code, examples, diagrams) MUST be original or properly attributed. External references cited with links and context. Code snippets that adapt patterns from external sources include attribution comments. No copy-paste from online tutorials without explicit rewriting and understanding.

### VI. Docusaurus-Compatible Format
All documentation MUST use standard Markdown (.md files only—NO MDX). Content structured for Docusaurus deployment with proper heading hierarchy, cross-references via relative links, and image asset management. Output MUST render correctly on GitHub Pages without custom preprocessing.

## Content & Structure Standards

### Coverage Requirements
Content MUST cover:
- Four core modules (topics/units)
- Capstone project (hands-on integration)
- Minimum 3 chapters/sections per module
- All examples runnable end-to-end

### Code Examples
- Every code example MUST include explanation before and after.
- Examples embedded with syntax highlighting and clear output expectations.
- All code snippets tested for correctness before inclusion.
- Dependencies and environment setup documented inline.

### Chatbot & RAG Requirements
- Embedded RAG chatbot MUST answer content-based questions accurately.
- Chatbot trained on book content; augmented with vetted external references.
- Query handling scope clearly defined (in-scope: book content + selected tech docs; out-of-scope: general advice outside domain).

## Development & Quality Standards

### Writing Process
- Outline and structure drafted first; approved before detailed writing.
- Technical accuracy reviewed via official sources before publication.
- Code examples tested in isolation; integration tested in capstone project.
- Peer review (or self-review against standards) before section finalization.

### Testing & Validation
- All runnable code sections MUST pass syntax validation.
- Setup procedures tested in clean environments (minimal baseline assumptions).
- Capstone project tested end-to-end; success criteria documented and validated.
- Integration between book content and RAG chatbot verified.

### Deployment Standards
- Final output deployed to GitHub Pages via Docusaurus build pipeline.
- All assets (images, code files, config) tracked in version control.
- Deployment process documented and reproducible (CI/CD or manual steps equally clear).
- Live site tested for accessibility, load times, and content correctness.

## Governance

### Amendment Procedure
Constitution changes require explicit ratification decision. MINOR amendments (clarifications, new guidance) documented as version bumps with rationale. MAJOR amendments (principle removals or redefinitions) require architectural review; document via ADR if cross-cutting.

### Versioning Policy
- MAJOR.MINOR.PATCH semantic versioning applies.
- MAJOR: Backward-incompatible principle changes or removals.
- MINOR: New principles, significant guidance expansion, structural additions.
- PATCH: Clarifications, wording refinements, typo fixes.

### Compliance & Review
- All feature specifications and plans MUST reference relevant principles.
- Pull requests affecting content quality or tooling decisions MUST justify alignment with constitution.
- Periodic review (end of each module) to validate adherence and adjust if experience contradicts principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-21 | **Last Amended**: 2025-12-21
