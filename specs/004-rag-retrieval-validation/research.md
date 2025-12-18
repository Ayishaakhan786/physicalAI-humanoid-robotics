# Research Findings: RAG Retrieval Pipeline & Validation

**Feature**: 004-rag-retrieval-validation
**Date**: 2025-12-18

## Overview of Research Tasks

The following research tasks were identified to address technical unknowns and inform detailed design decisions for the RAG retrieval pipeline validation.

## Findings

### R-001: Qdrant Client Querying Methods for Pre-embedded Vectors

-   **Objective**: Clarify the correct method and parameter naming for `QdrantClient` version 1.16.2 when performing a vector search using a pre-embedded query vector. Address the observed discrepancies between `search()`, `query()`, and `query_points()`.
-   **Decision**: [NEEDS RESEARCH: Specific `QdrantClient` method (e.g., `search()`, `query_points()`) and the exact parameter name for the query vector (e.g., `query_vector`, `vector`).]
-   **Rationale**: [NEEDS RESEARCH: Justification for chosen method based on API stability, performance, and clear parameter handling for pre-embedded vectors.]
-   **Alternatives Considered**: [NEEDS RESEARCH: Other Qdrant client versions, alternative wrapper functions for querying.]

### R-002: Best Practices for RAG Retrieval Evaluation

-   **Objective**: Identify common metrics and frameworks for assessing retrieval relevance and robustness, especially for Docusaurus-based technical content.
-   **Decision**: [NEEDS RESEARCH: Recommended metrics (e.g., Hit Rate, MRR, Precision@k) and a suitable Python library/framework for RAG evaluation (e.g., Ragas, LlamaIndex evaluation modules).]
-   **Rationale**: [NEEDS RESEARCH: Why chosen metrics/framework are appropriate for this project's goals and constraints.]
-   **Alternatives Considered**: [NEEDS RESEARCH: Custom evaluation scripts, manual relevance judgments, other evaluation toolkits.]

### R-003: Cohere Embedding Model Usage for Queries

-   **Objective**: Confirm the `input_type` parameter for the Cohere `embed-english-v3.0` model when embedding query strings (`search_query` vs. `classification` or `clustering`). Understand any specific parameters or optimizations for query embedding.
-   **Decision**: [NEEDS RESEARCH: Optimal `input_type` for Cohere query embedding, any specific Cohere API client nuances for query embedding.]
-   **Rationale**: [NEEDS RESEARCH: Justification for the chosen `input_type` to ensure embeddings are optimized for retrieval similarity.]
-   **Alternatives Considered**: [NEEDS RESEARCH: Other Cohere models for query embedding (if applicable), fine-tuning possibilities.]

### R-004: Handling Out-of-Scope Queries in Qdrant

-   **Objective**: Determine effective strategies for handling queries that are semantically unrelated to the indexed book content, to prevent returning irrelevant results or hallucinations.
-   **Decision**: [NEEDS RESEARCH: Methods like score thresholds (`score_threshold`), minimum number of matching documents (`min_should_match`), or filtering irrelevant metadata tags.]
-   **Rationale**: [NEEDS RESEARCH: How the chosen strategy effectively filters noise and indicates low relevance for out-of-scope queries.]
-   **Alternatives Considered**: [NEEDS RESEARCH: Post-retrieval re-ranking, LLM-based filtering of retrieved results.]

---
