# Feature Tasks: RAG Retrieval Pipeline & Validation

**Feature Branch**: `004-rag-retrieval-validation`
**Created**: 2025-12-18
**Status**: Draft

## Overview

This document outlines the actionable tasks for implementing the "RAG Retrieval Pipeline & Validation" feature, organized by phases and user stories. Each task is defined with a unique ID, status, and associated file paths to facilitate development and tracking. The tasks are ordered to reflect dependencies and promote incremental delivery.

## Phases

### Phase 1: Setup (Project Initialization for Retrieval Validation)

**Goal**: Establish initial files and ensure the development environment is ready for retrieval-specific work.

- [X] T001 Create `backend/src/retrieval_service.py` to house core retrieval logic and `backend/run_retrieval_tests.py` as the main script for the validation suite.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\retrieval_service.py`
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\run_retrieval_tests.py`
- [X] T002 Ensure existing Python project in `backend/` has a functional `uv` environment with all Spec-1 dependencies (`cohere`, `qdrant-client`, etc.) installed. (Verification task, assume done from previous phase).
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\pyproject.toml`
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\.venv\`
- [X] T003 Verify `backend/.env` file contains `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY` with valid credentials. (Verification task, assume user provided).
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\.env`

### Phase 2: Foundational (Common Components & Utilities)

**Goal**: Implement shared data models and refactor common functions to support the retrieval and validation logic.

- [X] T004 Refactor `embed_query` function from `backend/src/validation.py` to `backend/src/embedding.py` to centralize query embedding. Update imports in `backend/src/validation.py` and `backend/main.py`.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\embedding.py`
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\validation.py`
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\main.py`
- [X] T005 Create Pydantic models or data classes for `RetrievedResult` and `RetrievalTestCase` in `backend/src/models.py`.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\models.py`
- [X] T006 Implement a utility to load `RetrievalTestCase`s from a test data file (e.g., `backend/data/retrieval_test_cases.json`) in `backend/src/utils/test_data_loader.py`.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\utils\test_data_loader.py`
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\data\retrieval_test_cases.json`

### Phase 3: User Story 1 - Semantic Search for Book Content (P1)

**Goal**: Enable core semantic similarity search against the Qdrant collection.

**Independent Test**: Provide a natural language query to the retrieval pipeline and inspect the returned chunks for semantic relevance and correct ranking.

- [X] T007 [P] [US1] Implement `retrieve_chunks(qdrant_client: QdrantClient, collection_name: str, query_vector: List[float], limit: int = 5, min_score: float = 0.0, filters: Optional[models.Filter] = None) -> List[RetrievedResult]` in `backend/src/retrieval_service.py` using `qdrant_client.query_points` with the `vector` parameter.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\retrieval_service.py`
- [X] T008 [US1] Create initial `RetrievalTestCase`s for in-scope queries with expected `chunk_id`s/fragments and save to `backend/data/retrieval_test_cases.json`.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend/data/retrieval_test_cases.json`
- [X] T009 [US1] Integrate `embed_query` (from `embedding.py`) and `retrieve_chunks` into `backend/run_retrieval_tests.py` to demonstrate basic semantic search.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\run_retrieval_tests.py`

### Phase 4: User Story 2 - Metadata Integrity and Usability (P1)

**Goal**: Ensure retrieved content includes complete and accurate metadata for context and verification.

**Independent Test**: Execute queries and verify that each retrieved `RetrievedResult` object includes complete and accurate `EmbeddingMetadata` fields (URL, section, heading, chunk ID, etc.).

- [X] T010 [P] [US2] Verify and refine `retrieve_chunks` (T007) in `backend/src/retrieval_service.py` to ensure it correctly populates the `retrieved_chunk_metadata` field of `RetrievedResult` objects with all expected payload data.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\retrieval_service.py`
- [X] T011 [US2] Add test cases to `backend/data/retrieval_test_cases.json` specifically validating that retrieved chunks contain all expected metadata fields and their values are accurate, as per `spec.md` SC-003.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend/data/retrieval_test_cases.json`

### Phase 5: User Story 3 - Retrieval Pipeline Robustness and Performance (P2)

**Goal**: Validate the pipeline's robustness, efficiency, and accurate handling of various query types.

**Independent Test**: Run a comprehensive suite of diverse queries (in-scope, out-of-scope, edge cases), measure retrieval latency, and qualitatively assess result relevance, ranking, and absence of hallucinated content.

- [X] T012 [P] [US3] Implement `evaluate_retrieval(retrieved_results: List[RetrievedResult], test_case: RetrievalTestCase) -> Dict[str, Any]` in `backend/src/retrieval_service.py` to assess relevance, ranking, metadata match, and out-of-scope handling.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\retrieval_service.py`
- [X] T013 [US3] Implement `run_retrieval_test_suite(test_cases: List[RetrievalTestCase], qdrant_client: QdrantClient, cohere_api_key: str, collection_name: str) -> Dict[str, Any]:` in `backend/run_retrieval_tests.py` to orchestrate the execution and reporting of all test cases.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\run_retrieval_tests.py`
- [X] T014 [US3] Add a diverse set of test cases for out-of-scope queries and edge cases to `backend/data/retrieval_test_cases.json`, including expected behavior for low relevance/empty results.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend/data/retrieval_test_cases.json`
- [X] T015 [US3] Implement latency measurement within `retrieve_chunks` (T007) and aggregate reporting of average/P95 latency into `evaluate_retrieval` (T012) and `run_retrieval_test_suite` (T013).
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\retrieval_service.py`
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\run_retrieval_tests.py`
- [X] T016 [US3] Refine logging and detailed reporting of evaluation results within `run_retrieval_test_suite` in `backend/run_retrieval_tests.py`.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\run_retrieval_tests.py`

### Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Finalize documentation, ensure code quality, and address any remaining cross-cutting aspects for the retrieval validation.

- [X] T017 Update `backend/README.md` to include instructions for setting up and running the retrieval validation suite.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\README.md`
- [X] T018 Review code for modularity, reusability, adherence to Python best practices, and overall code quality across all new and modified modules (`retrieval_service.py`, `run_retrieval_tests.py`, `test_data_loader.py`).
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\`
- [X] T019 Add comments and comprehensive docstrings to all new public functions, methods, and classes.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\`
- [X] T020 Implement error handling and retry mechanisms in `retrieval_service.py` functions for Qdrant/Cohere interactions, similar to `main.py` from Spec-1.
    -   `D:\CODING\physicalAI-Humanoid-robotics\backend\src\retrieval_service.py`

## Dependency Graph (User Story Completion Order)

Phase 1 (Setup)
    ↓
Phase 2 (Foundational)
    ↓
Phase 3 (User Story 1: Semantic Search for Book Content)
    ↓
Phase 4 (User Story 2: Metadata Integrity and Usability)
    ↓
Phase 5 (User Story 3: Retrieval Pipeline Robustness and Performance)
    ↓
Phase 6 (Polish & Cross-Cutting Concerns)

## Parallel Execution Opportunities

- **Research Tasks**: Research tasks identified in `plan.md` (`R-001` to `R-004`) can be conducted in parallel with early development phases (Setup and Foundational) or amongst different team members.
- **Within User Stories**:
    - **US1 (Semantic Search)**: T007 (implementing `retrieve_chunks`) and T008 (creating initial test cases) can be done in parallel.
    - **US3 (Robustness & Performance)**: T012 (implementing `evaluate_retrieval`) can be developed in parallel with T007 (if a mock `retrieve_chunks` output is used). T014 (adding test cases) can run in parallel with other implementation tasks.

## Implementation Strategy

The implementation will follow an MVP-first approach, delivering each User Story as an independently testable increment.
- **MVP Scope**: User Story 1 (Semantic Search for Book Content) and User Story 2 (Metadata Integrity and Usability) together form the core MVP for the retrieval pipeline, demonstrating functional search with context.
- **Incremental Delivery**: User Story 3 (Retrieval Pipeline Robustness and Performance) will enhance the pipeline with comprehensive validation and performance measurement, building upon the MVP.
- **Continuous Integration**: Each phase will aim for working, tested code that integrates cleanly with previous phases.

## Task Completeness Validation

- All User Stories (P1, P2) from `spec.md` have dedicated phases with associated tasks.
- All required internal API functions (`retrieval_api.md`) are covered by implementation tasks.
- Key entities (`data-model.md`) are represented in implementation tasks (e.g., models, or their usage in functions).
- Setup, Foundational, and Polish tasks address cross-cutting concerns.
- Each task adheres to the checklist format: `- [ ] TXXX [P?] [USX?] Description with file path`.
