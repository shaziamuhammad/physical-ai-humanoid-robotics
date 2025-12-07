---
id: 0005
title: Generate Project Task Breakdown
stage: tasks
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-robotics-textbook
branch: master
user: User
command: /sp.tasks
labels: ["tasks", "project-management"]
links:
  spec: specs/physical-ai-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
- specs/physical-ai-robotics-textbook/tasks.md
tests:
- n/a
---

## Prompt

/sp.tasks

Using the approved Constitution, existing Specification, and the newly generated Plan, create a complete and professional task breakdown for the project:

“Physical AI & Humanoid Robotics — AI-Native Capstone Textbook”

Your task list MUST:

1. Be divided into logical phases based on the Plan:
   - Project Setup
   - Documentation Structure & Content Creation
   - Docusaurus Configuration
   - Homepage UI Development
   - Styling & Custom Theme
   - RAG-Friendly Content Formatting
   - Future RAG Backend (High-Level Tasks Only)
   - Future Bonus Features (High-Level Tasks Only)
   - Deployment Preparation

2. Each task MUST:
   - Begin with an action verb (Create, Configure, Implement, Write, Style, Organize, etc.)
   - Be specific and atomic (something CCR can execute)
   - Reference concrete files or components when relevant (e.g., docusaurus.config.ts, sidebars.ts, docs/*.md, index.tsx, custom.css)
   - Map directly back to the implementation plan

3. Include BOTH:
   - Executable tasks for Requirement 1 (book + UI + structure)
   - Planning-only tasks for Requirement 2 and bonus features (NOT implementation)

4. Include tasks for:
   - Creating all 7 required chapters
   - Structuring headings for RAG
   - Implementing homepage sections (Hero, Module Cards, Roadmap, Outcomes)
   - Ensuring TypeScript Docusaurus environment is correct
   - Setting up GitHub Pages readiness steps

5. DO NOT implement anything here.
   Only generate clear, actionable tasks that Claude Code / CCR can execute later during /sp.implement.

Output:
A complete, hierarchical, cleanly structured task list suitable for automated execution by Claude Code.

## Response snapshot

Generated the project task breakdown based on the provided requirements, constitution, specification, and plan, and saved it to specs/physical-ai-robotics-textbook/tasks.md.

## Outcome

- ✅ Impact: Project task breakdown generated.
- 🧪 Tests: n/a
- 📁 Files: Created specs/physical-ai-robotics-textbook/tasks.md
- 🔁 Next prompts: Awaiting further instructions
- 🧠 Reflection: Successfully generated the project task breakdown and recorded it.

## Evaluation notes (flywheel)

- Failure modes observed: n/a
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a
