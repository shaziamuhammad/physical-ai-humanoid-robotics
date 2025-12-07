---
id: 0002
title: Generate Project Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: User
command: /sp.constitution
labels: ["constitution", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:
- n/a
---

## Prompt

/sp.constitution

Project Name:
Physical AI & Humanoid Robotics — AI-Native Capstone Textbook

Purpose:
Create a clean, professional, academically structured AI-Native textbook for the Physical AI & Humanoid Robotics capstone. The book must serve as the core learning resource for the hackathon project, built entirely with Docusaurus (TypeScript) and designed for future integration with a RAG chatbot. All content must be accurate, minimal, clear, and deeply educational.

Scope:
- 7 structured chapters:
  1. Quarter Overview
  2. Module 1 — ROS 2: The Robotic Nervous System
  3. Module 2 — Digital Twin (Gazebo + Unity)
  4. Module 3 — NVIDIA Isaac: AI-Robot Brain
  5. Module 4 — t cards + roadmap section
- Accessible typography and spacing
- Fully responsive layout
- Professional academic visual identitVision-Language-Action (VLA)
  6. Why Physical AI Matters
  7. Learning Outcomes
- Modern customized UI for Docusaurus
- RAG-friendly grammar and chunking
- Clear engineering explanations suitable for robotics students
- Book-level consistency & professional formatting

Core Principles:
- Simplicity and clarity over unnecessary complexity
- Technical correctness based strictly on real robotics tools
- Minimalism in writing; avoid over-explanation
- Modular content for RAG (short sections, clear headings)
- Strong educational tone matching university-level materials
- Professional English writing quality
- No hallucinated APIs, commands, or technologies

Writing Quality Requirements:
- Every chapter must include definitions, concepts, and practical examples
- Sentences must be short, clear, and directly informative
- Paragraphs must be concise and chunked for RAG retrieval
- Use consistent terminology across all modules

UI/UX Requirements:
- Clean, modern, dark-themed UI
- Hero section + module highlighy

Constraints:
- Must work flawlessly on GitHub Pages free tier
- Must remain lightweight for embeddings
- No backend deployment in this step (backend comes in Requirement 2)
- No long or unnecessary code blocks—keep it essential

RAG Rules:
- RAG should answer ONLY from book text
- Content must be semantically chunkable
- Use section headings (H2, H3) consistently
- Avoid mixing unrelated concepts in one block
- Keep terminology stable and defined once per chapter

Success Criteria:
- Book builds successfully in Docusaurus TypeScript
- Content accuracy and consistency across chapters
- Professional writing quality suitable for hackathon evaluation
- Clean and attractive UI
- Structured, RAG-ready content suitable for Requirement 2 chatbot

Generate the full constitution.

## Response snapshot

Generated the project constitution based on the provided requirements and saved it to .specify/memory/constitution.md.

## Outcome

- ✅ Impact: Project constitution generated.
- 🧪 Tests: n/a
- 📁 Files: Updated .specify/memory/constitution.md
- 🔁 Next prompts: Awaiting further instructions
- 🧠 Reflection: Successfully generated the project constitution and recorded it.

## Evaluation notes (flywheel)

- Failure modes observed: n/a
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a
