# Implementation Plan: RAG Retrieval Pipeline & Validation

**Project Name**: RAG Chatbot - Spec-2: Retrieval Pipeline & Validation
**Feature Branch**: `004-rag-retrieval-validation`
**Created**: 2025-12-18
**Status**: Draft

## Problem Statement

AI engineers and evaluators need to validate the end-to-end retrieval pipeline for the Physical AI & Humanoid Robotics book. This involves querying embedded content stored in Qdrant and ensuring accurate, relevant document retrieval, including semantic similarity search, correct chunk retrieval, metadata integrity, and overall pipeline robustness, before integrating with a RAG agent.

## Proposed Solution

Develop a Python-based retrieval pipeline and a local test suite that will:
1.  **Load Qdrant Collection**: Connect to the existing `rag_embedding` Qdrant Cloud collection and verify its basic integrity (e.g., vector count).
2.  **Query Embedding**: Utilize the Cohere `embed-english-v3.0` model to embed natural language queries for similarity search.
3.  **Semantic Similarity Search**: Perform semantic similarity searches against the Qdrant collection to retrieve relevant text chunks.
4.  **Result Validation**: Implement mechanisms to validate the relevance, ranking, metadata integrity, and reproducibility of retrieved chunks for various test prompts (in-scope, out-of-scope, edge cases).
5.  **Performance Measurement**: Measure retrieval latency to ensure acceptable performance for interactive usage.
6.  **Reporting**: Log and report validation results for review by AI engineers and evaluators.

---

## Technical Context

### Key Technologies & Services
-   **Language**: Python
-   **Embedding Provider**: Cohere (`embed-english-v3.0` model, consistent with Spec-1)
-   **Vector Database**: Qdrant Cloud (existing `rag_embedding` collection)
-   **Environment/Dependency Management**: `uv` (reusing from Spec-1)

### Constraints
-   Use existing Qdrant Cloud collection (`rag_embedding`).
-   Embeddings (for queries) must match Spec-1 configuration (Cohere `embed-english-v3.0` model).
-   Retrieval must be reproducible and deterministic.
-   All tests executed locally via Python.
-   Retrieval implementation should be modular and reusable.

### Assumptions
-   The `rag_embedding` Qdrant collection is already populated with relevant book content embeddings as a result of Spec-1.
-   Cohere API access and a valid API key will be provisioned/available.
-   Qdrant Cloud access and a valid API key will be provisioned/available.
-   The Python environment (with `uv` and required libraries) from Spec-1 is set up and functional.
-   The semantic chunking strategy and metadata enrichment from Spec-1 are appropriate for retrieval needs.
-   "Acceptable retrieval latency" will be defined by an explicit threshold (e.g., <500ms for 95% of queries).

---

## Constitution Check

The `constitution.md` file in `.specify/memory/` appears to be a template and does not define concrete project principles. Therefore, a full constitution check cannot be performed at this stage. It is assumed that once project-specific constitutional principles are established, this plan will be reviewed against them. (Refer to ADR-0001: Undefined Project Constitution).

---

## Gates

### Initial Gate: Specification Review
-   **Status**: PASS
-   **Justification**: The feature specification (`spec.md`) for "RAG Retrieval Pipeline & Validation" has been reviewed. All requirements are clear, testable, and free of `[NEEDS CLARIFICATION]` markers. User stories are well-defined, and success criteria are measurable and technology-agnostic.

---

## Phases

### Phase 0: Outline & Research

**Goal**: To resolve technical unknowns regarding Qdrant client usage for querying, retrieval evaluation methods, and performance optimization.

#### Research Tasks

-   **R-001**: Research Qdrant Client querying methods for pre-embedded vectors:
    -   Focus: Clarify the correct method and parameter naming for `QdrantClient` version 1.16.2 when performing a vector search (e.g., `search()`, `query()`, `query_points()`) given the recent ambiguities observed.
    -   Focus: Understand how to effectively apply filters (e.g., by `document_section`, `source_url`) during retrieval.
    -   Focus: Investigate methods for optimizing query performance on Qdrant Free Tier.
-   **R-002**: Investigate best practices for RAG retrieval evaluation:
    -   Focus: Identify common metrics (e.g., precision, recall, hit rate, MRR) for assessing retrieval relevance.
    -   Focus: Research frameworks or methodologies for creating ground truth test sets for Docusaurus content.
    -   Focus: Explore methods for evaluating robustness against out-of-scope or adversarial queries.
-   **R-003**: Cohere embedding model usage for queries:
    -   Focus: Confirm `embed-english-v3.0` model's `input_type` for queries (`search_query` vs. `classification`).
    -   Focus: Understand any specific parameters or optimizations for query embedding.

#### Consolidated Findings (`research.md`)

*This section will be populated upon completion of the research tasks.*

---

### Phase 1: Design & Contracts

**Prerequisites**: All research tasks from Phase 0 are completed, and `research.md` is populated.

**Goal**: To define the data structures, internal function interfaces, and provide a quickstart guide for the retrieval and validation components.

#### Data Model (`data-model.md`)

Based on the `Key Entities` identified in the feature specification, the following data model extensions and usages will be considered:

-   **Query**:
    -   `query_text` (string): The natural language query from the user.
    -   `query_vector` (List[float]): The Cohere embedding of `query_text`.
-   **RetrievedResult**: (Pydantic model representing a Qdrant search hit)
    -   `chunk_id` (string): Unique ID of the retrieved chunk.
    -   `score` (float): Relevance score from Qdrant.
    -   `payload` (EmbeddingMetadata): The associated metadata for the chunk.
    -   `text` (string): The actual text content of the chunk.
-   **RetrievalTestCase**:
    -   `query` (string): The test query.
    -   `expected_chunk_ids` (List[string]): List of chunk IDs expected to be highly relevant.
    -   `expected_fragments` (List[string], optional): List of text fragments expected in top results.
    -   `min_score_threshold` (float, optional): Minimum score for a result to be considered relevant.
    -   `is_out_of_scope` (bool): True if this query should return no relevant results.

#### Internal API Contracts (Function Signatures)

These functions will form the modular components of the retrieval and validation pipeline. Some might extend existing modules or be new.

-   `def embed_query(query: str, cohere_api_key: str) -> List[float]:` (Existing in `validation.py`, possibly move to `embedding.py` for reusability)
    -   **Description**: Generates a vector embedding for a single query string using Cohere.
    -   **Input**: `query` (str), `cohere_api_key` (str).
    -   **Output**: `List[float]` (query vector).
-   `def retrieve_chunks(qdrant_client: QdrantClient, collection_name: str, query_vector: List[float], limit: int = 5, min_score: float = 0.0) -> List[RetrievedResult]:`
    -   **Description**: Performs a semantic similarity search in Qdrant and returns ranked retrieved chunks with their metadata.
    -   **Input**: `qdrant_client`, `collection_name`, `query_vector`, `limit`, `min_score` (optional threshold).
    -   **Output**: `List[RetrievedResult]`.
-   `def evaluate_retrieval(retrieved_results: List[RetrievedResult], test_case: RetrievalTestCase) -> Dict[str, Any]:`
    -   **Description**: Evaluates the relevance and correctness of retrieved results against a defined test case.
    -   **Input**: `retrieved_results`, `test_case`.
    -   **Output**: `Dict[str, Any]` (evaluation metrics like `is_relevant`, `rank_of_expected`, `metadata_match`).
-   `def run_retrieval_test_suite(test_cases: List[RetrievalTestCase], qdrant_client: QdrantClient, cohere_api_key: str, collection_name: str) -> Dict[str, Any]:`
    -   **Description**: Orchestrates the execution of a suite of retrieval test cases.
    -   **Input**: `test_cases`, `qdrant_client`, `cohere_api_key`, `collection_name`.
    -   **Output**: `Dict[str, Any]` (overall test results, summaries).

#### Quickstart (`quickstart.md`)

*This will be generated in a separate step after the initial plan.md is complete.* Will focus on how to set up and run the retrieval test suite.

---

### Agent Context Update

*This will be done after `Phase 1` content is generated.*

---

I will now write this content to `specs/004-rag-retrieval-validation/plan.md`.