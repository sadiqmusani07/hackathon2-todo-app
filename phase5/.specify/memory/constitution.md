<!-- SYNC IMPACT REPORT:
Version change: 1.1.0 → 1.2.0
Modified principles: Updated RAG system requirements and infrastructure constraints
Added sections: None
Removed sections: None
Templates requiring updates: ✅ .specify/templates/plan-template.md, ✅ .specify/templates/spec-template.md, ✅ .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Specification-First Development
All features and functionality must be defined in specifications before implementation. Specifications serve as the single source of truth for the project, ensuring deterministic and auditable development. Every major feature (book structure, chatbot behavior, APIs, data flows) must be defined using Spec-Kit Plus before any code is written.

### II. Deterministic, Auditable AI-Assisted Authoring
Development must leverage Claude Code for all content generation and refinement, following approved specifications. All AI-generated content must be reviewed for correctness and consistency. The development process must be fully traceable through Prompt History Records (PHRs) to ensure accountability and reproducibility.

### III. Content-Grounded Intelligence (NON-NEGOTIABLE)
The RAG chatbot must provide responses that are strictly derived only from indexed book content with no hallucinations. Both global book Q&A and user-selected text-only Q&A modes must maintain strict grounding to prevent generation of unverified information.

### IV. Modular, Reproducible Architecture
The system must maintain clear separation between content (Docusaurus), infrastructure (deployment), and AI logic (RAG system). The architecture must be modular to enable independent development, testing, and deployment of components while maintaining system integrity.

### V. Production Readiness
All components must be designed for production deployment with observability, maintainability, and scalability considerations. The system must be deployable, observable, and maintainable with clear runbooks, monitoring, and error handling.

### VI. Free-Tier Infrastructure Compatibility
All infrastructure choices must remain compatible with free-tier services (Qdrant Cloud, Neon Serverless Postgres) to ensure accessibility and cost-effectiveness. No proprietary or undocumented APIs may be used that would compromise long-term maintainability.

## Additional Constraints

### Book Authoring Standards
- Content must be written using Docusaurus (MDX-based structure) with clear technical tone for professional developers and AI practitioners
- Examples must be executable or directly actionable with step-by-step instructions
- All content must be generated and refined via Claude Code following approved specifications
- Clear table of contents, navigation, architecture diagrams, and setup instructions must be provided

### RAG System Requirements
- Backend implemented with FastAPI and OpenAI Agents/ChatKit SDKs
- Vector storage using Qdrant Cloud (Free Tier)
- Metadata and session/state storage using Neon Serverless Postgres
- Strict grounding enforcement: responses must derive only from indexed book content
- Support for both global book Q&A and user-selected text-only Q&A modes

### Documentation Requirements
- Clear table of contents and navigation
- Architecture diagrams (conceptual, not proprietary)
- Step-by-step setup and deployment instructions
- API and data-flow explanations for the RAG system
- Explicit limitations and failure modes of the chatbot must be documented

## Development Workflow

### Implementation Process
- Specifications must precede implementation and remain synchronized throughout development
- All major features must be defined using Spec-Kit Plus before implementation begins
- Each user story must be independently testable and deliver value when implemented alone
- Code reviews must verify compliance with all constitutional principles
- All changes must be small, testable, and reference code precisely

### Quality Gates
- All AI-generated content must be reviewed for correctness and consistency
- No proprietary or undocumented APIs may be used
- No unverifiable claims or undocumented behaviors allowed
- All chatbot responses must be fully grounded in book content
- Project must be cloneable, deployable, and verifiable by third parties without additional guidance

## Governance

The constitution supersedes all other development practices and must be strictly followed. Amendments require formal documentation, approval process, and migration plan. All PRs and code reviews must verify compliance with constitutional principles. The project must maintain adherence to specification-first development, content-grounded intelligence, and production readiness standards.

**Version**: 1.2.0 | **Ratified**: 2025-06-13 | **Last Amended**: 2025-12-19