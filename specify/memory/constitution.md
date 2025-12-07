# Project Constitution: AI-Native Capstone Textbook

## Project Name: Physical AI & Humanoid Robotics — AI-Native Capstone Textbook

## Purpose:
Create a clean, professional, academically structured AI-Native textbook for the Physical AI & Humanoid Robotics capstone. The book must serve as the core learning resource for the hackathon project, built entirely with Docusaurus (TypeScript) and designed for future integration with a RAG chatbot. All content must be accurate, minimal, clear, and deeply educational.

## Scope:
### In Scope:
- 7 structured chapters:
  1. Quarter Overview
  2. Module 1 — ROS 2: The Robotic Nervous System
  3. Module 2 — Digital Twin (Gazebo + Unity)
  4. Module 3 — NVIDIA Isaac: AI-Robot Brain
  5. Module 4 — Vision-Language-Action (VLA)
  6. Why Physical AI Matters
  7. Learning Outcomes
- Modern customized UI for Docusaurus
- RAG-friendly grammar and chunking
- Clear engineering explanations suitable for robotics students
- Book-level consistency & professional formatting
- Accessible typography and spacing
- Fully responsive layout
- Professional academic visual identity

### Out of Scope:
- Backend deployment (to be addressed in Requirement 2)
- Long or unnecessary code blocks
- Unrelated content or features not directly contributing to the textbook's purpose

## Core Principles:
- **Simplicity and Clarity:** Prioritize straightforward explanations over unnecessary complexity.
- **Technical Correctness:** Content must be strictly accurate, based on real robotics tools and established engineering principles.
- **Minimalism:** Avoid over-explanation; writing should be concise and direct.
- **Modular Content:** Design sections for RAG retrieval with short, focused paragraphs and clear headings.
- **Educational Tone:** Maintain a professional, academic voice suitable for university-level materials.
- **Professional English:** Ensure high-quality writing, grammar, and style.
- **No Hallucinations:** Never invent APIs, commands, technologies, or concepts; all information must be verifiable.

## Writing Quality Requirements:
- **Chapter Structure:** Every chapter must include definitions, concepts, and practical examples.
- **Sentence Clarity:** Sentences must be short, clear, and directly informative.
- **Paragraph Conciseness:** Paragraphs must be concise and semantically chunked for RAG retrieval.
- **Consistent Terminology:** Use consistent terminology across all modules and chapters.

## UI/UX Requirements:
- **Design:** Clean, modern, dark-themed UI.
- **Key Sections:** Include a hero section and module highlights.

## Constraints:
- **Deployment:** Must work flawlessly on GitHub Pages free tier.
- **Performance:** Must remain lightweight for efficient embeddings.
- **Backend:** No backend deployment in this initial step.
- **Code Blocks:** No long or unnecessary code blocks; keep them essential and illustrative.

## RAG Rules:
- **Content Source:** RAG must answer ONLY from the textbook text.
- **Semantic Chunking:** Content must be semantically chunkable for effective retrieval.
- **Consistent Headings:** Use section headings (H2, H3) consistently to structure content.
- **Concept Isolation:** Avoid mixing unrelated concepts within a single content block.
- **Stable Terminology:** Maintain stable terminology, with definitions provided once per chapter.

## Success Criteria:
- **Build Success:** The book must build successfully using Docusaurus (TypeScript).
- **Content Integrity:** High accuracy and consistency of content across all chapters.
- **Professional Quality:** Writing quality must be professional and suitable for hackathon evaluation.
- **User Interface:** Clean, attractive, and functional UI.
- **RAG Readiness:** Structured, RAG-ready content suitable for Requirement 2 chatbot integration.
