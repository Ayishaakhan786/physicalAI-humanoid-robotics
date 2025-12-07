# Tasks: Physical AI & Humanoid Robotics Book

**Input**: Design documents from `/specs/001-robotics-book-spec/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The feature specification implies a strong need for testing and reproducibility, so relevant test tasks are included within the QA steps.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create Docusaurus project skeleton, including `/book/docs/`, `/book/src/`, `/book/static/`, `book/docusaurus.config.js`, `book/package.json`, `book/sidebars.js` (populated), and `book/README.md` with build/deploy steps. Output: `book/` directory and its initial contents.
- [x] T002 Create GitHub Actions workflow for Docusaurus deployment to GitHub Pages. Output: `.github/workflows/deploy-docusaurus.yml`.

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core content planning and structure that MUST be complete before content generation for any user story.

- [x] T003 Define high-level book layout (table of contents) including 4 modules, Preface, Hardware Appendix, Capstone Appendix, References, Glossary. For each module, list 4-6 chapter titles, a 1-sentence summary, target word count (800-2000), and recommended filename. Output: JSON structure.
- [x] T004 For each chapter identified in T003, create a detailed chapter specification in Markdown (≤ 400-800 words), including title, filename, learning objectives (3-5), required pre-knowledge, high-level lab tasks, code snippets (list of files and short description), diagrams (text description + suggested filename), tests/validation criteria, and APA sources. Output: `specs/001-robotics-book-spec/module-XX/<chapter-filename>.md` for each chapter.

## Phase 3: User Story 1 - Learn ROS 2 Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Deliver the complete content for "Module 1 - The Robotic Nervous System (ROS 2)", including theory, labs, code examples, and related assets, fully tested and deployable.

**Independent Test**: The Docusaurus site builds successfully with the ROS 2 module content. All ROS 2 code examples build and run on Ubuntu 22.04 with ROS 2 Humble. The chapter tests/validation criteria pass.

### Implementation for User Story 1

- [x] T005 [US1] Generate Markdown/MDX content for "Module 1 - ROS 2" (Chapter 1: Intro) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-01-ros2/01-intro.md`. (Word target: 800-1200 words).
- [x] T006 [US1] Generate Markdown/MDX content for "Module 1 - ROS 2" (Chapter 2: ROS Architecture) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-01-ros2/02-ros-architecture.md`. (Word target: 800-1200 words).
- [x] T007 [US1] Generate Markdown/MDX content for "Module 1 - ROS 2" (Chapter 3: Nodes & Topics) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-01-ros2/03-nodes-topics.md`. (Word target: 800-1200 words).
- [x] T008 [US1] Generate Markdown/MDX content for "Module 1 - ROS 2" (Chapter 4: Services & Actions) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-01-ros2/04-services-actions.md`. (Word target: 800-1200 words).
- [x] T009 [US1] Generate Markdown/MDX content for "Module 1 - ROS 2" (Chapter 5: Basic Robot Control) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-01-ros2/05-robot-control.md`. (Word target: 800-1200 words).
- [x] T010 [P] [US1] Create code examples (ROS 2 packages) for "Module 1 - ROS 2" based on chapter specs. Output: `book/src/ros2_examples/`.
- [x] T011 [P] [US1] Create diagrams and visual assets for "Module 1 - ROS 2" based on chapter specs. Output: `book/docs/assets/`.
- [x] T012 [US1] QA Review and Technical Verification for "Module 1 - ROS 2" content and code. (Run ROS 2 commands locally, check code lint, verify APA citations, Docusaurus build test).

## Phase 4: User Story 2 - Explore Digital Twin Concepts (Priority: P1)

**Goal**: Deliver the complete content for "Module 2 - The Digital Twin (Gazebo & Unity)", including theory, labs, code examples, and related assets, fully tested and deployable.

**Independent Test**: The Docusaurus site builds successfully with the Digital Twin module content. All simulation code examples (Gazebo) build and run on Ubuntu 22.04. The chapter tests/validation criteria pass.

### Implementation for User Story 2

- [x] T013 [US2] Generate Markdown/MDX content for "Module 2 - Digital Twin" (Chapter 1: Intro) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-02-digital-twin/01-intro.md`. (Word target: 800-1200 words).
- [x] T014 [US2] Generate Markdown/MDX content for "Module 2 - Digital Twin" (Chapter 2: Gazebo Simulation) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-02-digital-twin/02-gazebo-sim.md`. (Word target: 800-1200 words).
- [x] T015 [US2] Generate Markdown/MDX content for "Module 2 - Digital Twin" (Chapter 3: URDF & Robot Models) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-02-digital-twin/03-urdf-models.md`. (Word target: 800-1200 words).
- [x] T016 [US2] Generate Markdown/MDX content for "Module 2 - Digital Twin" (Chapter 4: Advanced Gazebo) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-02-digital-twin/04-advanced-gazebo.md`. (Word target: 800-1200 words).
- [x] T017 [US2] Generate Markdown/MDX content for "Module 2 - Digital Twin" (Chapter 5: Unity Robotics Hub) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-02-digital-twin/05-unity-robotics.md`. (Word target: 800-1200 words).
- [x] T018 [P] [US2] Create code examples (Gazebo/Unity simulations) for "Module 2 - Digital Twin" based on chapter specs. Output: `book/src/gazebo_sims/` and `book/src/unity_sims/`.
- [x] T019 [P] [US2] Create diagrams and visual assets for "Module 2 - Digital Twin" based on chapter specs. Output: `book/docs/assets/`.
- [x] T020 [US2] QA Review and Technical Verification for "Module 2 - Digital Twin" content and code. (Run Gazebo/Unity simulations locally, check code lint, verify APA citations, Docusaurus build test).

## Phase 5: User Story 3 - Utilize NVIDIA Isaac Platform (Priority: P2)

**Goal**: Deliver the complete content for "Module 3 - The AI-Robot Brain (NVIDIA Isaac)", including theory, labs, code examples, and related assets, fully tested and deployable.

**Independent Test**: The Docusaurus site builds successfully with the NVIDIA Isaac module content. All Isaac Sim code examples run correctly within Isaac Sim. The chapter tests/validation criteria pass.

### Implementation for User Story 3

- [x] T021 [US3] Generate Markdown/MDX content for "Module 3 - NVIDIA Isaac" (Chapter 1: Intro) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-03-isaac/01-intro.md`. (Word target: 800-1200 words).
- [x] T022 [US3] Generate Markdown/MDX content for "Module 3 - NVIDIA Isaac" (Chapter 2: Isaac Sim Basics) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-03-isaac/02-isaac-sim-basics.md`. (Word target: 800-1200 words).
- [x] T023 [US3] Generate Markdown/MDX content for "Module 3 - NVIDIA Isaac" (Chapter 3: Robotics with Isaac) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-03-isaac/03-robotics-isaac.md`. (Word target: 800-1200 words).
- [x] T024 [US3] Generate Markdown/MDX content for "Module 3 - NVIDIA Isaac" (Chapter 4: Perception & AI) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-03-isaac/04-perception-ai.md`. (Word target: 800-1200 words).
- [x] T025 [US3] Generate Markdown/MDX content for "Module 3 - NVIDIA Isaac" (Chapter 5: Advanced Isaac) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-03-isaac/05-advanced-isaac.md`. (Word target: 800-1200 words).
- [x] T026 [P] [US3] Create code examples (Isaac Sim scenes/scripts) for "Module 3 - NVIDIA Isaac" based on chapter specs. Output: `book/src/isaac_sims/`.
- [x] T027 [P] [US3] Create diagrams and visual assets for "Module 3 - NVIDIA Isaac" based on chapter specs. Output: `book/docs/assets/`.
- [x] T028 [US3] QA Review and Technical Verification for "Module 3 - NVIDIA Isaac" content and code. (Run Isaac Sim examples locally, check code lint, verify APA citations, Docusaurus build test).

## Phase 6: User Story 4 - Implement Vision and Language Models (VLA) (Priority: P2)

**Goal**: Deliver the complete content for "Module 4 - Vision-Language-Action (VLA)", including theory, labs, code examples, and related assets, fully tested and deployable.

**Independent Test**: The Docusaurus site builds successfully with the VLA module content. All VLA integration code examples run correctly, demonstrating ASR and LLM capabilities. The chapter tests/validation criteria pass.

### Implementation for User Story 4

- [x] T029 [US4] Generate Markdown/MDX content for "Module 4 - VLA" (Chapter 1: Intro) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-04-vla/01-intro.md`. (Word target: 800-1200 words).
- [x] T030 [US4] Generate Markdown/MDX content for "Module 4 - VLA" (Chapter 2: Speech Recognition) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-04-vla/02-speech-recognition.md`. (Word target: 800-1200 words).
- [x] T031 [US4] Generate Markdown/MDX content for "Module 4 - VLA" (Chapter 3: Natural Language Understanding) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-04-vla/03-nlu.md`. (Word target: 800-1200 words).
- [x] T032 [US4] Generate Markdown/MDX content for "Module 4 - VLA" (Chapter 4: Action & Planning) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-04-vla/04-action-planning.md`. (Word target: 800-1200 words).
- [x] T033 [US4] Generate Markdown/MDX content for "Module 4 - VLA" (Chapter 5: VLA Capstone) based on its chapter spec. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/module-04-vla/05-vla-capstone.md`. (Word target: 800-1200 words).
- [x] T034 [P] [US4] Create code examples (VLA integrations) for "Module 4 - VLA" based on chapter specs. Output: `book/src/vla_integrations/`.
- [x] T035 [P] [US4] Create diagrams and visual assets for "Module 4 - VLA" based on chapter specs. Output: `book/docs/assets/`.
- [x] T036 [US4] QA Review and Technical Verification for "Module 4 - VLA" content and code. (Run VLA examples locally, check code lint, verify APA citations, Docusaurus build test).

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final quality checks.

- [x] T037 Generate content for Preface. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/preface.md`.
- [x] T038 Generate content for Hardware Appendix. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/hardware-appendix.md`.
- [x] T039 Generate content for Capstone Appendix. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/capstone-appendix.md`.
- [x] T040 Generate content for References and Glossary. **Instruction**: Generate full chapter content following book outline, academic tone (simple for students), tables/text-based diagrams, real-world examples, publish-ready, smooth transitions, subheadings, micro-sections, exercises. Output: `book/docs/references.md`, `book/docs/glossary.md`.
- [x] T041 Create the QA checklist and acceptance test plan. Output: `specs/001-robotics-book-spec/qa-checklist.md`, `specs/001-robotics-book-spec/test-commands.csv`.
- [x] T042 Final review and editing of all book content, ensuring consistency and adherence to APA.
- [x] T043 Full end-to-end testing of the capstone demo pipeline.
- [x] T044 Run Docusaurus broken links check.

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Content generation tasks for a module's chapters (e.g., T005-T009) can be performed sequentially or in parallel.
- Code example creation (e.g., T010) and diagram creation (e.g., T011) can be performed in parallel with content generation, but should be complete before QA review (e.g., T012).
- QA review (e.g., T012) depends on all content, code, and diagrams for that module being generated.

### Parallel Opportunities

- All Setup tasks (Phase 1) marked [P] can run in parallel.
- Once Foundational phase (Phase 2) completes, user stories (Phase 3-6) can be worked on in parallel by different team members.
- Within each user story, content generation for different chapters can be parallelized.
- Code and diagram creation tasks can be parallelized with content generation within a story.

---

## Parallel Example: User Story 1

```bash
# Example of parallel execution for content generation, code, and diagrams:
Task: "T005 [US1] Generate Markdown/MDX content for "Module 1 - ROS 2" (Chapter 1: Intro) based on its chapter spec. Output: book/docs/module-01-ros2/01-intro.md"
Task: "T006 [US1] Generate Markdown/MDX content for "Module 1 - ROS 2" (Chapter 2: ROS Architecture) based on its chapter spec. Output: book/docs/module-01-ros2/02-ros-architecture.md"
Task: "T010 [P] [US1] Create code examples (ROS 2 packages) for "Module 1 - ROS 2" based on chapter specs. Output: book/src/ros2_examples/"
Task: "T011 [P] [US1] Create diagrams and visual assets for "Module 1 - ROS 2" based on chapter specs. Output: book/docs/assets/"
```

---

## Implementation Strategy

### MVP First (Prioritizing P1 User Stories)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1 (Learn ROS 2 Fundamentals)
4. **STOP and VALIDATE**: Test User Story 1 independently.
5. If ready, proceed to Phase 4: User Story 2 (Explore Digital Twin Concepts).

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP for ROS 2 content!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories.

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together.
2. Once Foundational is done:
   - Developer A: User Story 1 content and associated tasks.
   - Developer B: User Story 2 content and associated tasks.
   - Developer C: User Story 3 content and associated tasks.
   - Developer D: User Story 4 content and associated tasks.
3. Stories complete and integrate independently.

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Verify tests fail before implementing
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
