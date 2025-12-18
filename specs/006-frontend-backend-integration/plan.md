# Implementation Plan: Frontend and Backend Integration

**Project Name**: RAG Chatbot - Spec-4: Frontend and Backend Integration
**Feature Branch**: `006-frontend-backend-integration`
**Created**: 2025-12-18
**Status**: Draft

## Problem Statement

The agent-based RAG backend needs to be integrated with the Docusaurus frontend to enable in-book question answering. This requires establishing secure communication, embedding a React-based chatbot UI inside the published Docusaurus book, and enabling context-aware Q&A using both full book content and user-selected text, all while ensuring smooth operation in local and deployed environments.

## Proposed Solution

Develop a React-based chatbot UI component that is compatible with Docusaurus. This component will be embedded within the Docusaurus book, allowing users to input questions. The frontend will communicate via REST (JSON over HTTP) with the FastAPI backend's `/query` endpoint. It will capture user-selected text to provide context-aware questions to the backend. The backend URL will be configurable via environment variables, and the UI will be designed to integrate seamlessly without breaking the Docusaurus build or modifying existing book content.

---

## Technical Context

### Key Technologies & Services
-   **Frontend Framework**: React (used by Docusaurus)
-   **Documentation Site Generator**: Docusaurus
-   **Backend API**: FastAPI (`/query` endpoint from Spec-3)
-   **Communication Protocol**: REST (JSON over HTTP)
-   **Environment Variables**: For backend URL configuration.

### Constraints
-   Frontend built using React (Docusaurus compatible).
-   Communication via REST (JSON over HTTP).
-   No direct OpenAI calls from frontend.
-   Backend URL must be configurable via environment variables.
-   UI must not modify or break existing book content.

### Assumptions
-   The Docusaurus site structure (version 2.x or 3.x) allows for embedding custom React components in a maintainable way (e.g., via swizzling or custom plugins/themes).
-   The FastAPI backend (Spec-3) is deployed and accessible at a configurable, secure URL (HTTPS in production).
-   CORS configuration on the FastAPI backend permits requests from the Docusaurus frontend's origin.
-   Secure communication (HTTPS) in deployed environments will be handled by deployment infrastructure (e.g., Vercel, Netlify, Nginx proxy) and not require complex frontend-specific security implementations beyond standard web practices.
-   User-selected text capture can be implemented using standard browser APIs (e.g., `window.getSelection()`) without needing external libraries that might conflict with Docusaurus.
-   Styling will leverage Docusaurus's existing CSS variables or allow for custom styling without overriding critical Docusaurus components.

---

## Constitution Check

The `constitution.md` file in `.specify/memory/` appears to be a template and does not define concrete project principles. Therefore, a full constitution check cannot be performed at this stage. It is assumed that once project-specific constitutional principles are established, this plan will be reviewed against them. (Refer to ADR-0001: Undefined Project Constitution).

---

## Gates

### Initial Gate: Specification Review
-   **Status**: PASS
-   **Justification**: The feature specification (`spec.md`) for "Frontend and Backend Integration" has been reviewed. All requirements are clear, testable, and free of `[NEEDS CLARIFICATION]` markers. User stories are well-defined, and success criteria are measurable and technology-agnostic.

---

## Phases

### Phase 0: Outline & Research

**Goal**: To resolve technical unknowns regarding Docusaurus integration, React component development within Docusaurus, and secure frontend-backend communication.

#### Research Tasks

-   **R-001**: Research Docusaurus custom component integration:
    -   Focus: Best practices for embedding custom React components (e.g., chatbot UI) into a Docusaurus site without using swizzling heavily. Options include custom plugins, themes, or simply placing components in designated areas.
    -   Focus: How to manage component state and interact with Docusaurus's context (e.g., current page URL).
-   **R-002**: Investigate methods for capturing user-selected text:
    -   Focus: Reliable cross-browser methods using JavaScript's `window.getSelection()` API.
    -   Focus: How to trigger an action (e.g., open chatbot with pre-filled context) from a text selection event.
-   **R-003**: Research secure frontend-backend communication:
    -   Focus: CORS configuration requirements on the FastAPI backend (review existing).
    -   Focus: Safely configuring the backend API URL in the React frontend for both local development and deployed environments (e.g., Docusaurus environment variables, `process.env`).
    -   Focus: Best practices for handling API requests (e.g., `fetch` API, Axios) and displaying loading/error states.
-   **R-004**: Docusaurus build system compatibility:
    -   Focus: Ensuring custom React components and their dependencies do not interfere with the Docusaurus build process or output.
    -   Focus: Testing for potential conflicts with Docusaurus's internal React version or styling.

#### Consolidated Findings (`research.md`)

*This section will be populated upon completion of the research tasks.*

---

### Phase 1: Design & Contracts

**Prerequisites**: All research tasks from Phase 0 are completed, and `research.md` is populated.

**Goal**: To design the frontend component, define data flow, and provide a quickstart guide for integration.

#### Data Model (`data-model.md`)

Based on the `Key Entities` identified in the feature specification and existing backend contracts:

-   **FrontendQueryRequest**:
    -   `question` (string): User's typed question.
    -   `selected_text_context` (string, optional): User-selected text from the page.
    -   `current_page_url` (HttpUrl): The URL of the Docusaurus page where the question originated.
-   **FrontendChatState**:
    -   `is_chat_open` (bool): Whether the chat UI is visible.
    -   `messages` (List[ChatMessage]): History of questions and answers.
    -   `current_input` (string): Current text in the input field.
    -   `is_loading` (bool): True when waiting for backend response.
    -   `error_message` (string, optional): Displayed error message.
-   **ChatMessage**:
    -   `type` (enum: 'user', 'agent'): Sender of the message.
    -   `content` (string): Message text (question or answer).
    -   `sources` (List[Source], optional): Sources for agent answers.
    -   `timestamp` (datetime): When the message was sent/received.

#### Internal API Contracts (Frontend-Specific)

This section defines the client-side interaction with the backend API.

-   **`postQuery(question: string, selectedText?: string, pageUrl?: string) -> Promise<AgentResponse>`**:
    -   **Description**: Function within the frontend to send a query to the FastAPI backend.
    -   **Input**: `question`, optional `selectedText`, optional `pageUrl`.
    -   **Output**: `Promise<AgentResponse>` (resolved AgentResponse or rejected with ErrorResponse).
    -   **Behavior**: Handles HTTP request, JSON serialization/deserialization, and basic error mapping.

#### Quickstart (`quickstart.md`)

*This will be generated in a separate step after the initial plan.md is complete.* Will focus on how to set up the Docusaurus project, embed the chatbot UI, configure environment variables, and run the local development server.

---

### Agent Context Update

*This will be done after `Phase 1` content is generated.*

---

I will now write this content to `specs/006-frontend-backend-integration/plan.md`.