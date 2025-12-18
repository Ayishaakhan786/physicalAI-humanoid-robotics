---
description: "Task list for Frontend and Backend Integration feature"
---

# Tasks: Frontend and Backend Integration

**Input**: Design documents from `/specs/006-frontend-backend-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `book/src/`
- **Backend**: `backend/src/`
- **Docusaurus Config**: `book/`

---

## Phase 1: Research

**Purpose**: To resolve technical unknowns before starting implementation.

- [X] T001 [P] Research and decide on the best Docusaurus integration method (custom plugin, theme swizzling, etc.) and document in `specs/006-frontend-backend-integration/research.md`.
- [X] T002 [P] Research and decide on a reliable method for capturing user-selected text using `window.getSelection()` and document in `specs/006-frontend-backend-integration/research.md`.
- [X] T003 [P] Research and document the specific CORS headers required for the FastAPI backend and how to handle Docusaurus environment variables for the `RAG_BACKEND_URL` in `specs/006-frontend-backend-integration/research.md`.
- [X] T004 [P] Research and document potential Docusaurus build system conflicts and solutions in `specs/006-frontend-backend-integration/research.md`.

---

## Phase 2: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure.

- [X] T005 Navigate to the Docusaurus project `book/` and run `npm install` to ensure all dependencies are present.
- [X] T006 Create a `.env` file in `book/` for local development and add `RAG_BACKEND_URL=http://localhost:8000/query`.
- [X] T007 [P] Create the basic directory structure for the chatbot component: `book/src/components/ChatbotUI/`.

---

## Phase 3: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

- [X] T008 Create a frontend API service module `book/src/services/api.js` to handle communication with the backend.
- [X] T009 In `book/src/services/api.js`, implement the `postQuery` function to send a `POST` request to the backend, handling JSON serialization and basic error responses.

---

## Phase 4: User Story 1 - Ask a Question from the Book (Priority: P1) 🎯 MVP

**Goal**: Allows a user to ask a general question about the book content and see the answer.

**Independent Test**: Open the Docusaurus site, use the chatbot to ask a question, and verify a grounded answer with sources is displayed.

### Implementation for User Story 1

- [X] T010 [US1] Create the main `ChatbotUI` React component in `book/src/components/ChatbotUI/index.js`.
- [X] T011 [US1] Implement the basic UI for the chatbot, including a message display area, a text input, and a send button.
- [X] T012 [US1] Implement state management for the chatbot using React hooks (`useState`, `useReducer`) in `book/src/components/ChatbotUI/index.js` to manage `is_chat_open`, `messages`, `current_input`, `is_loading`, and `error_message`.
- [X] T013 [US1] Integrate the `postQuery` service from `book/src/services/api.js` into the `ChatbotUI` component to send a user's question and receive the response.
- [X] T014 [US1] Render the conversation history (`messages` state) in the `ChatbotUI` component, distinguishing between 'user' and 'agent' messages.
- [X] T015 [US1] Display a loading indicator while `is_loading` is true.
- [X] T016 [US1] Display any error messages from the backend.

---

## Phase 5: User Story 2 - Context-Aware Question Answering (Priority: P1)

**Goal**: Enable users to ask questions about specific text they have selected on the page.

**Independent Test**: Select text on a Docusaurus page, ask a question, and verify the answer is based on the selected context.

### Implementation for User Story 2

- [X] T017 [US2] Implement a mechanism to detect and capture user-selected text using `window.getSelection()` within the Docusaurus page context.
- [X] T018 [US2] Add a UI element (e.g., a small button or context menu) that appears when text is selected, allowing the user to initiate a context-aware query.
- [X] T019 [US2] Modify the `postQuery` call in the `ChatbotUI` component to include the `selected_text_context` when a context-aware query is made.
- [X] T020 [US2] Update the `ChatbotUI` to pre-fill the question or associate the selected context with the next query.

---

## Phase 6: User Story 3 - Seamless Integration with Docusaurus (Priority: P2)

**Goal**: Ensure the chatbot UI integrates into the Docusaurus site without breaking builds or content.

**Independent Test**: Run `docusaurus build` successfully and verify the deployed site works as expected.

### Implementation for User Story 3

- [X] T021 [US3] Embed the `ChatbotUI` component into the Docusaurus site using the method decided in `T001`. For example, by swizzling `theme-classic/Layout`.
- [X] T022 [US3] Ensure the backend URL is configurable via environment variables in `docusaurus.config.js` and properly accessed in `book/src/services/api.js` (e.g., via `process.env.RAG_BACKEND_URL`).
- [ ] T023 [US3] Run the `npm run build` command in the `book/` directory and verify it completes without errors.
- [ ] T024 [US3] Test the deployed/built site to ensure the chatbot does not interfere with existing Docusaurus content, styles, or functionality.
- [ ] T025 [P] [US3] Add styling for the `ChatbotUI` component in `book/src/components/ChatbotUI/styles.css` ensuring it is visually consistent with the Docusaurus theme and does not bleed into other components.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

- [ ] T026 [P] Add documentation for the `ChatbotUI` component and its usage in `book/docs/chatbot-integration.md`.
- [ ] T027 Perform final validation by following the steps in `specs/006-frontend-backend-integration/quickstart.md`.
- [ ] T028 Code cleanup and refactoring of the `ChatbotUI` component.

---

## Dependencies & Execution Order

- **Research (Phase 1)**: Should be completed first to inform implementation choices.
- **Setup (Phase 2)** & **Foundational (Phase 3)**: Must be completed before user story implementation.
- **User Stories (Phase 4-6)**: Can be implemented in priority order. US1 is the MVP.
- **Polish (Phase 7)**: Final step.
