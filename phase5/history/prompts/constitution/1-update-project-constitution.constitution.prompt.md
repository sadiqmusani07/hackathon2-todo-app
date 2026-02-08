---
id: 1
title: Update project constitution
stage: constitution
date: 2025-12-19
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: sadiqmusani07
command: /sp.constitution
labels: ["constitution", "spec-driven", "project-setup", "documentation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

/sp.constitution

Project: AI/Spec-Driven Book with Embedded RAG Chatbot

Objective:
Design, implement, and publish a unified technical book project using AI-native development practices. The project must result in a publicly deployed book with an integrated Retrieval-Augmented Generation (RAG) chatbot capable of answering questions grounded strictly in the book's content.

Core principles:
- Specification-first development (Spec-Kit Plus as the single source of truth)
- Deterministic, auditable AI-assisted authoring (Claude Code)
- Content-grounded intelligence (no hallucinations in book or chatbot)
- Modular, reproducible architecture (clear separation of content, infra, and AI logic)
- Production readiness (deployable, observable, maintainable)

Key standards:
- Book authoring:
  - Written using Docusaurus (MDX-based structure)
  - Content generated and refined via Claude Code following approved specs
  - Clear technical tone for professional developers and AI practitioners
  - Examples must be executable or directly actionable
- Specifications:
  - All major features (book structure, chatbot behavior, APIs, data flows) defined using Spec-Kit Plus
  - Specs must precede implementation and remain synchronized
- RAG chatbot:
  - Built using OpenAI Agents/ChatKit SDKs
  - Backend implemented with FastAPI
  - Vector storage: Qdrant Cloud (Free Tier)
  - Metadata + session/state storage: Neon Serverless Postgres
  - Strict grounding: responses must be derived only from indexed book content
  - Context mode:
    - Global book Q&A
    - User-selected text–only Q&A (answers limited to selected passages)
- Deployment:
  - Book deployed to GitHub Pages
  - CI/CD workflow documented and reproducible

Constraints:
- Book scope: end-to-end explanation of the system (concepts, architecture, implementation, deployment)
- All AI-generated content must be reviewed for correctness and consistency
- No proprietary or undocumented APIs
- No unverifiable claims or undocumented behaviors
- Free-tier compatible infrastructure only

Documentation requirements:
- Clear table of contents and navigation
- Architecture diagrams (conceptual, not proprietary)
- Step-by-step setup and deployment instructions
- API and data-flow explanations for the RAG system
- Explicit limitations and failure modes of the chatbot

Success criteria:
- Book successfully builds with Docusaurus and deploys to GitHub Pages
- RAG chatbot is embedded and functional within the published book
- Chatbot answers are fully grounded in book content
- Selected-text Q&A mode works as specified
- All specs are implemented and traceable to delivered features
- Project can be cloned, deployed, and verified by a third party without additional guidance

## Response snapshot

Successfully updated the project constitution with specific principles for the AI/Spec-Driven Book with Embedded RAG Chatbot project, replacing all template placeholders with concrete values based on the project specifications.

## Outcome

- ✅ Impact: Project constitution updated with specific principles for AI/Spec-Driven Book with RAG Chatbot
- 🧪 Tests: none
- 📁 Files: 1 file updated (.specify/memory/constitution.md)
- 🔁 Next prompts: Continue with spec development using /sp.specify
- 🧠 Reflection: The constitution now properly reflects the project's core principles and constraints.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification development
