# AI/Spec-Driven Book Creation Constitution

<!--
SYNC IMPACT REPORT:
Version Change: 0.0.0 → 1.0.0
Modified Principles: All (initial creation)
Added Sections:
  - I. Spec-First Development
  - II. AI-Native Workflow
  - III. Maximum Truth-Seeking
  - IV. Progressive Enhancement
  - V. Open-Source Excellence
  - VI. Accessibility & Standards
  - VII. Performance & Constraints
  - Development Workflow
  - Quality Gates
Templates Requiring Updates:
  ✅ plan-template.md (Constitution Check section already references constitution)
  ✅ spec-template.md (requirements and success criteria align with principles)
  ✅ tasks-template.md (task organization supports principle-driven development)
Follow-up TODOs: None
-->

## Core Principles

### I. Spec-First Development

Every piece of content, chapter, and feature MUST originate from an approved spec in the `/specs` directory.

**Rules:**
- No content may be written without a corresponding spec file
- Specs MUST be approved before implementation begins
- All specs reside in `/specs/<feature-name>/` with clear approval status
- Changes to published content require spec amendments

**Rationale:** Spec-first development ensures all content is purposeful, reviewed, and traceable. It prevents scope creep and maintains quality through deliberate planning.

### II. AI-Native Workflow

All prose, code examples, diagrams (Mermaid), and frontmatter are primarily generated or refined by Claude Code (or equivalent frontier model), then human-reviewed and committed.

**Rules:**
- AI tools (Claude Code, Grok-4, Claude 3.5+) are first-class content creators
- All AI-generated content MUST undergo human review before merging
- Human review focuses on accuracy, clarity, and alignment with specs
- Record all significant AI exchanges in Prompt History Records (PHRs)

**Rationale:** Leveraging AI for content generation accelerates production while maintaining quality through human oversight. This hybrid approach combines AI efficiency with human judgment.

### III. Maximum Truth-Seeking

Technical explanations MUST be precise, up-to-date (2025–2026 best practices), and reflect real-world usability.

**Rules:**
- Code examples MUST be tested and runnable with latest Node.js LTS
- Technical claims require citations to official docs, research papers, or canonical repos
- Outdated information MUST be flagged and updated
- Ambiguous or uncertain content MUST be clarified or marked as NEEDS CLARIFICATION
- Flesch-Kincaid readability: grade 8–10

**Rationale:** Readers trust technical books to be accurate and current. Misinformation undermines credibility and wastes reader time. Truth-seeking ensures the book remains a reliable reference.

### IV. Progressive Enhancement

Content MUST be readable and useful even with JavaScript disabled; Docusaurus features (search, dark mode, versioning) are enhancements, not requirements.

**Rules:**
- Core content accessible without JavaScript
- Navigation, hierarchy, and reading flow work in plain HTML
- Interactive features (search, syntax highlighting, diagrams) enhance but don't block access
- All interactive elements have accessible fallbacks

**Rationale:** Progressive enhancement ensures maximum accessibility and resilience. Readers on limited devices or restrictive networks can still access core content.

### V. Open-Source Excellence

Zero proprietary lock-in, permissive licensing (MIT or CC-BY-4.0 where applicable), and easy forking/contribution.

**Rules:**
- All code licensed under MIT
- All prose licensed under CC-BY-4.0
- No paid fonts, icons, or assets (Google Fonts, Font Awesome Free, Docusaurus built-ins only)
- No external hosting dependencies except GitHub Pages (free tier)
- Repository MUST include CONTRIBUTING.md and SPEC-WRITING.md

**Rationale:** Open-source principles ensure longevity, community engagement, and educational accessibility. Removing barriers to contribution builds a sustainable project.

### VI. Accessibility & Standards

All content MUST meet WCAG 2.1 AA compliance standards.

**Rules:**
- Alt text required for all images and diagrams
- Proper heading hierarchy (h1 → h2 → h3, no skips)
- Sufficient color contrast (4.5:1 for normal text, 3:1 for large text)
- Keyboard navigation fully supported
- Screen reader compatible
- Valid Docusaurus frontmatter for all .mdx files

**Rationale:** Accessibility is a fundamental right, not a feature. WCAG compliance ensures readers with disabilities can fully access and benefit from the book.

### VII. Performance & Constraints

All content MUST meet strict performance and resource budgets.

**Rules:**
- Total repository size: < 500 MB (including images and generated site)
- Build time on GitHub Actions: < 8 minutes
- Lighthouse scores (mobile): Performance ≥ 90, Accessibility ≥ 95, Best Practices ≥ 95, SEO ≥ 95
- Page load time: < 3 seconds on 3G connection
- Images optimized (WebP or similar, < 200 KB each)

**Rationale:** Performance constraints ensure fast load times and broad device compatibility. Resource limits keep hosting free and builds efficient.

## Development Workflow

### Content Creation Process

1. **Specification Phase:**
   - Create feature spec in `/specs/<feature-name>/spec.md`
   - Define user scenarios, requirements, and success criteria
   - Obtain approval (mark status: approved)

2. **Planning Phase:**
   - Generate implementation plan in `/specs/<feature-name>/plan.md`
   - Document technical decisions and architecture
   - Identify architectural decisions requiring ADRs

3. **Implementation Phase:**
   - Generate content using AI tools (Claude Code)
   - Follow tasks defined in `/specs/<feature-name>/tasks.md`
   - Test all code examples with latest Node.js LTS
   - Validate all links (zero dead links)

4. **Review Phase:**
   - Human review for accuracy, clarity, and alignment
   - Accessibility audit (WCAG 2.1 AA)
   - Performance validation (Lighthouse scores)
   - Link validation (Lychee or Docusaurus link checker)

5. **Deployment Phase:**
   - Commit changes with clear messages
   - CI/CD pipeline runs build and validation
   - Deploy to GitHub Pages
   - Verify live site functionality

### Documentation Standards

- **Writing Style:** Clear, engaging, professional yet approachable (Flesch-Kincaid grade 8–10)
- **Tone:** Helpful mentor assuming intermediate programming knowledge but new to AI-native workflows
- **Code Examples:** Runnable, tested, including both TypeScript and JavaScript where meaningful
- **Diagrams:** Exclusively Mermaid.js (live editable)
- **Citations:** BibTeX + Citation.js; link to official docs, research papers (arXiv, ACL, NeurIPS), or canonical repos
- **Frontmatter:** Valid Docusaurus v3+ frontmatter (title, description, sidebar_position, keywords, hide_table_of_contents)

### Testing & Validation

- **Code Examples:** All code MUST be tested with latest Node.js LTS
- **Links:** Zero dead links (validated via Lychee or Docusaurus link checker)
- **Accessibility:** WCAG 2.1 AA compliance validated
- **Performance:** Lighthouse scores meet or exceed minimums
- **Build:** Successful build in < 8 minutes on GitHub Actions
- **End-to-End:** At least one external reader successfully replicates AI-native workflow

## Quality Gates

All features MUST pass these gates before merging to main:

1. **Spec Approval Gate:**
   - Spec exists in `/specs/<feature-name>/spec.md`
   - Status marked as "approved" or "implemented"

2. **Code Quality Gate:**
   - All code examples tested and runnable
   - No syntax errors or broken examples
   - TypeScript and JavaScript variants provided where applicable

3. **Accessibility Gate:**
   - Alt text on all images/diagrams
   - Proper heading hierarchy
   - WCAG 2.1 AA color contrast
   - Screen reader compatibility verified

4. **Performance Gate:**
   - Lighthouse scores: Performance ≥ 90, Accessibility ≥ 95, Best Practices ≥ 95, SEO ≥ 95
   - Build time < 8 minutes
   - Repository size < 500 MB

5. **Link Validation Gate:**
   - Zero dead links (internal or external)
   - All references point to live, authoritative sources

6. **Documentation Gate:**
   - CONTRIBUTING.md and SPEC-WRITING.md present and complete
   - All specs in `/specs` directory marked with clear status

## Governance

This constitution supersedes all other practices and conventions in the project.

**Amendment Process:**
- Amendments require documentation of rationale and impact
- Version MUST be incremented according to semantic versioning:
  - **MAJOR:** Backward incompatible governance/principle removals or redefinitions
  - **MINOR:** New principle/section added or materially expanded guidance
  - **PATCH:** Clarifications, wording, typo fixes, non-semantic refinements
- All dependent templates and documentation MUST be updated for consistency
- Migration plan required for breaking changes

**Compliance & Enforcement:**
- All PRs/reviews MUST verify compliance with constitution principles
- Quality gates MUST be passed before merge
- Complexity and deviations MUST be justified in writing
- Constitution violations block merges unless explicitly justified and approved

**Continuous Improvement:**
- Constitution reviewed quarterly for relevance and clarity
- Feedback from contributors incorporated via amendment process
- Lessons learned from implementation documented in ADRs

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
