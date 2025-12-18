# Feature Tasks: Agent-Based Retrieval & Answering Backend

**Feature Branch**: `005-rag-agent-backend`
**Created**: 2025-12-18
**Status**: Draft

## Overview

This document outlines the actionable tasks for implementing the "Agent-Based Retrieval & Answering Backend" feature, organized by phases and user stories. Each task is defined with a unique ID, status, and associated file paths to facilitate development and tracking. The tasks are ordered to reflect dependencies and promote incremental delivery.

## Phases

### Phase 1: Setup (Project Initialization for RAG Backend)

**Goal**: Establish necessary project dependencies and verify core environment configuration.

- [X] T001 Update `pyproject.toml` in `backend/` with new dependencies (`fastapi`, `uvicorn`, `openai-agents-sdk`, `openai`).
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\pyproject.toml`
- [X] T002 Run `uv sync` in `backend/` to install all new dependencies.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\`
- [X] T003 Verify `backend/.env` file contains `OPENAI_API_KEY` (in addition to existing `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`) with valid credentials. (Verification task, assume user provided).
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\.env`

### Phase 2: Foundational (Common Components & Agent Structure)

**Goal**: Define core data models for API interaction and set up the basic structure for agent tools and the RAG agent itself.

- [X] T004 Add Pydantic models for `QueryRequest`, `Source`, `AgentResponse`, `ErrorResponse` to `backend/src/models.py`.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\models.py`
- [X] T005 Create `backend/src/agent_tools.py` to define custom tools that the OpenAI RAG agent will utilize (e.g., `retrieve_documents`).
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\agent_tools.py`
- [X] T006 Create `backend/src/rag_agent.py` to define the RAG agent's orchestration logic, including its initialization and system prompt.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\rag_agent.py`

### Phase 3: User Story 1 - Ask a Grounded Question (P1)

**Goal**: Implement the core agent functionality for retrieving context and generating grounded answers to user questions.

**Independent Test**: Send an in-scope question to the agent and verify it returns an accurate, grounded answer with source citations.

- [X] T007 [P] [US1] Implement the `retrieve_documents` custom tool in `backend/src/agent_tools.py`. This tool should utilize the Spec-2 validated `embed_query` and `retrieve_chunks` functions.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\agent_tools.py`
- [X] T008 [P] [US1] Define the RAG agent's system prompt within `backend/src/rag_agent.py`, emphasizing strict context grounding and instructing the agent to use the `retrieve_documents` tool.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\rag_agent.py`
- [X] T009 [US1] Implement the agent's core answering logic within `backend/src/rag_agent.py`, integrating the `retrieve_documents` tool and generating grounded responses from retrieved context.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\rag_agent.py`
- [X] T010 [US1] Create initial test cases for in-scope questions in `backend/data/agent_test_cases.json` (or similar file) with expected grounded answers and source references.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend/data/agent_test_cases.json`

### Phase 4: User Story 2 - Handle Insufficient Context (P1)

**Goal**: Ensure the RAG agent explicitly refuses to answer or indicates insufficient context when necessary, preventing hallucinations.

**Independent Test**: Ask out-of-scope questions or questions with no relevant retrieved context and verify the agent's refusal behavior.

- [X] T011 [P] [US2] Refine the agent's system prompt and internal logic in `backend/src/rag_agent.py` to explicitly handle scenarios where retrieved context is insufficient or irrelevant, leading to a refusal to answer.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\rag_agent.py`
- [X] T012 [US2] Add test cases for out-of-scope questions and questions with deliberately insufficient retrieved context to `backend/data/agent_test_cases.json`, expecting a clear refusal message from the agent.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend/data/agent_test_cases.json`

### Phase 5: User Story 3 - API Interaction and Structured Responses (P2)

**Goal**: Expose the RAG agent's capabilities via a robust FastAPI `/query` endpoint returning structured JSON.

**Independent Test**: Make various HTTP POST requests to the `/query` endpoint and verify structured JSON responses, including correct answers, sources, and error handling.

- [X] T013 [P] [US3] Initialize the FastAPI application in `backend/main.py`, create an `app` instance, and configure it to expose a single `/query` HTTP POST endpoint.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\main.py`
- [X] T014 [P] [US3] Implement the `/query` endpoint logic in `backend/main.py` to accept `QueryRequest` (Pydantic model), invoke the RAG agent (from `rag_agent.py`), and return an `AgentResponse` (Pydantic model).
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\main.py`
- [X] T015 [US3] Implement structured error handling within the FastAPI application for invalid inputs (e.g., empty question, 400 Bad Request) and internal agent/pipeline failures (e.g., 500 Internal Server Error), returning `ErrorResponse`.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\main.py`
- [X] T016 [US3] Create a basic test file `backend/tests/test_api.py` to validate API contract adherence for the `/query` endpoint (request/response schemas, error handling).
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\tests\test_api.py`

### Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Finalize documentation, ensure code quality, and address remaining cross-cutting aspects for the RAG backend.

- [X] T017 Update `backend/README.md` to include instructions for setting up and running the FastAPI backend (using Uvicorn) and testing the `/query` endpoint.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\README.md`
- [X] T018 Review code for modularity, reusability, adherence to Python best practices, and overall code quality across all new and modified modules.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\`
- [X] T019 Add comments and comprehensive docstrings to all new public functions, methods, and classes.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\`
- [X] T020 Ensure all environment variables (including `OPENAI_API_KEY`) are securely loaded (via `config.py`) and not hardcoded.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\utils\config.py`

## Dependency Graph (User Story Completion Order)

Phase 1 (Setup)
    ↓
Phase 2 (Foundational)
    ↓
Phase 3 (User Story 1: Ask a Grounded Question)
    ↓
Phase 4 (User Story 2: Handle Insufficient Context)
    ↓
Phase 5 (User Story 3: API Interaction and Structured Responses)
    ↓
Phase 6 (Polish & Cross-Cutting Concerns)

## Parallel Execution Opportunities

-   **Research Tasks**: Research tasks identified in `plan.md` (`R-001` to `R-004`) can be conducted in parallel with early development phases (Setup and Foundational) or amongst different team members.
-   **Within User Stories**:
    -   **US1 (Grounded Question)**: T007 (implementing `retrieve_documents` tool) and T008 (defining system prompt) can be developed in parallel.
    -   **US3 (API Interaction)**: T013 (initializing FastAPI) and T016 (creating API tests) can be developed in parallel.

## Implementation Strategy

The implementation will follow an MVP-first approach, delivering each User Story as an independently testable increment.
-   **MVP Scope**: User Story 1 (Ask a Grounded Question) and User Story 2 (Handle Insufficient Context) form the core MVP for the RAG agent's intelligence.
-   **Incremental Delivery**: User Story 3 (API Interaction and Structured Responses) exposes this core functionality via a production-ready API.
-   **Continuous Integration**: Each phase will aim for working, tested code that integrates cleanly with previous phases.

## Task Completeness Validation

-   All User Stories (P1, P2) from `spec.md` have dedicated phases with associated tasks.
-   All required internal API functions (`query_api.md`) are covered by implementation tasks.
-   Key entities (`data-model.md`) are represented in implementation tasks (e.g., models, or their usage in functions).
-   Setup, Foundational, and Polish tasks address cross-cutting concerns.
-   Each task adheres to the checklist format: `- [ ] TXXX [P?] [USX?] Description with file path`
