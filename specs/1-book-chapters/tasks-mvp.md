# Tasks: Book MVP (2-3 Hour Scope)


**Goal**: Deliver working Docusaurus site with one complete chapter (ROS 2 fundamentals) as proof of concept
**Timeline**: 2-3 hours total

**Success Criteria**:
- Docusaurus builds and deploys to GitHub Pages
- Chapter 1 (ROS 2 intro) complete with 2 code examples
- All links and citations verified
- Ready to expand to full book

---

## Phase 1: Setup (45 minutes)

### Docusaurus Initialization

- [ ] T001 Create `docusaurus.config.js` at repository root (title: "Physical AI & Humanoid Robotics", baseUrl: "/native-book/")
- [ ] T002 Create `sidebars.js` with basic structure: Module 1 (ROS 2) only
- [ ] T003 Create `package.json` with Docusaurus 3.x dependencies
- [ ] T004 Create folder structure: `docs/`, `static/`, `docker/`
- [ ] T005 Run `npm install && npm run build` - verify build succeeds (0 errors)

**Output**: Working Docusaurus project that builds successfully

---

## Phase 2: Content Foundation (30 minutes)

### Templates & Setup

- [ ] T006 Create `docs/index.md` - book landing page (title, 4-5 sentences overview, link to Chapter 1)
- [ ] T007 Create `specs/1-book-chapters/contracts/chapter-template.md` - simple markdown template for chapters
- [ ] T008 Create `README.md` at repo root with quick start (Docker command, native setup links)
- [ ] T009 Create `docker/Dockerfile` with ROS 2 Humble + Python 3.11 (basic, pre-built)

**Output**: Templates ready; Docker image defined; README for readers

---

## Phase 3: MVP Chapter - ROS 2 Fundamentals (60-75 minutes)

### Chapter 1: ROS 2 Architecture (Nodes & Topics)

**Single chapter covering**: What is ROS 2? Pub/sub messaging. Running first node.

#### Content Writing (25 minutes)

- [ ] T010 Write `docs/01-module-1-ros2/01-introduction.md` covering:
  - Why ROS 2? (1 paragraph)
  - What is middleware? (2 paragraphs)
  - Pub/Sub pattern explained (2 paragraphs with diagram reference)
  - What you'll learn (bullet list)
  - All claims cited to official ROS 2 docs (https://docs.ros.org/en/humble/)

**Content Length**: ~800 words

#### Code Examples (25 minutes)

- [ ] T011 Create `docs/01-module-1-ros2/examples/01-simple-publisher.py` - basic pub/sub publisher in rclpy (20 lines, inline comments)
- [ ] T012 Create `docs/01-module-1-ros2/examples/01-simple-subscriber.py` - basic subscriber (20 lines, inline comments)
- [ ] T013 Create `docs/01-module-1-ros2/examples/01-example-output.txt` - expected output when running examples

#### Integration into Chapter (10 minutes)

- [ ] T014 Add code examples to `01-introduction.md` in format: setup → code → output → explanation (2-3 paragraphs per example)
- [ ] T015 Add inline citations (APA format) for all ROS 2 concepts linking to official docs
- [ ] T016 Verify chapter markdown renders correctly in Docusaurus (local preview)

**Output**: One complete chapter with 2 code examples, verified rendering

---

## Phase 4: Validation & Deployment (30 minutes)

### Testing & Build

- [ ] T017 Create `tests/test_chapter1.py` - pytest test that:
  - Verifies Chapter 1 markdown exists
  - Validates code examples have proper syntax (Python syntax check)
  - Confirms all links in chapter are valid URLs (check against official ROS 2 docs)

- [ ] T018 Run pytest: `pytest tests/test_chapter1.py` - all tests pass ✅

- [ ] T019 Run Docusaurus build: `npm run build` - verify 0 warnings, `build/` directory created ✅

- [ ] T020 Create `.github/workflows/deploy.yml` - GitHub Actions workflow for GitHub Pages deployment

### Deployment

- [ ] T021 Push code to GitHub and trigger GitHub Actions

- [ ] T022 Verify live site: Check Chapter 1 renders at `https://yourusername.github.io/native-book/`

- [ ] T023 Smoke test: Click links, verify code block syntax highlighting works, chapter is readable ✅

**Output**: Live GitHub Pages site with Chapter 1 deployed

---

## Phase 5: Documentation (15 minutes)

### Proof of Concept Summary

- [ ] T024 Create `NEXT_STEPS.md` documenting:
  - What was completed (Chapter 1, Docusaurus setup)
  - How to add Chapter 2 (copy template, follow same pattern)
  - Estimated time for full book (13-15 weeks concurrent)
  - Parallel opportunities for team expansion

- [ ] T025 Update `README.md` with "MVP Complete" badge and link to deployed site

**Output**: Clear roadmap for expanding to full book

---

## Summary: 187 Tasks → 25 MVP Tasks

| Phase | Full Plan | MVP | Reduction |
|-------|-----------|-----|-----------|
| Setup | 15 tasks | 5 tasks | 67% ↓ |
| Foundational | 12 tasks | 3 tasks | 75% ↓ |
| Chapters | 133 tasks | 16 tasks | 88% ↓ |
| Polish | 27 tasks | 1 task | 96% ↓ |
| **Total** | **187 tasks** | **25 tasks** | **87% ↓** |

---

## MVP vs Full Plan

### What's Included in MVP

✅ Working Docusaurus site (builds, deploys to GitHub Pages)
✅ One complete chapter (ROS 2 fundamentals)
✅ 2 code examples with explanation
✅ All citations verified
✅ Live deployment
✅ Clear template for expansion

### What's NOT in MVP (for future phases)

❌ Modules 2-4 (Digital Twin, Isaac, VLA)
❌ Capstone project integration
❌ 40+ additional code examples
❌ Docker/reproducibility setup
❌ Cross-platform testing
❌ RAG chatbot integration

---

## MVP Timeline

| Task Range | Phase | Time | Cumulative |
|-----------|-------|------|-----------|
| T001-T005 | Setup | 45 min | 45 min |
| T006-T009 | Foundation | 30 min | 75 min |
| T010-T016 | Chapter 1 | 60 min | 135 min |
| T017-T023 | Testing & Deploy | 30 min | 165 min |
| T024-T025 | Summary | 15 min | **180 min (3h)** |

---

## MVP Success Criteria ✅

- [ ] Docusaurus builds with 0 errors
- [ ] Chapter 1 (ROS 2 fundamentals) complete (~800 words)
- [ ] 2 code examples with output and explanation
- [ ] All citations link to official ROS 2 documentation
- [ ] Live GitHub Pages site deployed and accessible
- [ ] Chapter renders correctly (no markdown errors)
- [ ] Code examples have proper syntax highlighting
- [ ] One working chapter proves full book is feasible

---

## After MVP: Expansion Path

Once MVP is complete (3 hours):

**Next 5 hours**: Add Chapter 2-3 (ROS 2 module complete) - 2h per chapter
**Next 15 hours**: Add Modules 2-4 (Digital Twin, Isaac, VLA) - 5h per module
**Next 10 hours**: Capstone integration - full system
**Final 10 hours**: Polish, testing, deployment

**Total to full book**: 3h (MVP) + 40h (content) = **~43 hours solo** (or 13-15 hours concurrent team)

---

## How to Execute MVP

```bash
# 1. Setup (45 min)
cd native-book
npm install
npm run build

# 2. Content (60 min)
# Write docs/index.md
# Write docs/01-module-1-ros2/01-introduction.md
# Create examples/*.py files

# 3. Test & Deploy (30 min)
pytest tests/test_chapter1.py
npm run build
git push  # triggers GitHub Actions

# 4. Verify (15 min)
# Check live site at GitHub Pages URL
# Verify Chapter 1 renders correctly

# Done! MVP complete in ~2.5-3 hours
```

---

## Key Takeaways

- **MVP proves concept** with one working chapter
- **Fast feedback loop** - live site in 3 hours
- **Clear expansion path** - add chapters incrementally
- **Template-based** - Chapter 2-5 follow same pattern as Chapter 1
- **Parallel ready** - once Chapter 1 done, team can work on Modules 2-4 in parallel

Start here. Iterate. Expand.

