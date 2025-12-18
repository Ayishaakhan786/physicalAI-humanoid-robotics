# ADR-0008: Qdrant Querying Mechanism for Pre-embedded Vectors

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Proposed
- **Date:** 2025-12-18
- **Feature:** 004-rag-retrieval-validation
- **Context:** The retrieval pipeline's core function is to perform semantic similarity searches in the `rag_embedding` Qdrant Cloud collection using pre-embedded query vectors. Previous attempts to implement this functionality (`/sp.implement` for Spec-1) encountered significant challenges and confusion regarding the correct `QdrantClient` method and parameter names (e.g., `search`, `query`, `query_points`, and parameter names `query_vector`, `vector`). A clear and definitive decision on the precise Qdrant client API usage for vector search is critical to ensure a functional, robust, and maintainable retrieval component that correctly interfaces with the Qdrant Cloud.

## Decision

The Qdrant `query_points` method will be used for performing semantic similarity searches with pre-embedded query vectors. The parameter for passing the query vector will be explicitly named `vector` (e.g., `qdrant_client.query_points(collection_name=..., query_filter=..., vector=query_vector, limit=..., with_payload=...)`). This decision is based on diagnostic testing during previous implementation attempts where `qdrant_client.query_points` was identified as the correct method from the `dir()` output of the `QdrantClient` instance, and `vector` was inferred through trial and error as the expected parameter name for the installed `qdrant-client` version (1.16.2). Filters on payload metadata will be applied using the `query_filter` parameter with Qdrant's `models.Filter` object.

## Consequences

### Positive

*   **Functional Retrieval**: Provides a clear and working path to implement the core vector search functionality, resolving previous implementation blockers and ensuring the pipeline can proceed.
*   **Direct Vector Usage**: Allows for direct use of Cohere-generated `query_vector`s, avoiding any unnecessary re-embedding by the client or internal embedding, thus maintaining control over the embedding process.
*   **Metadata Filtering**: The `query_filter` parameter enables advanced retrieval logic by allowing filters on `EmbeddingMetadata` fields (e.g., filtering by `document_section` or `source_url`), which is crucial for precise RAG.
*   **Client Version Compatibility**: Aligns with the observed API of the currently installed `qdrant-client` version (1.16.2), ensuring the code runs without immediate attribute or argument errors.

### Negative

*   **API Volatility/Ambiguity**: This decision highlights the apparent volatility or lack of clear documentation/examples for specific `qdrant-client` versions, requiring careful attention to future client library updates and potentially leading to re-learning the API.
*   **Learning Curve**: Developers new to the project may need to navigate the nuances of the `qdrant-client` API to understand this specific choice, especially given the differing behaviors of `search`, `query`, and `query_points` methods.
*   **Potential for Future Renaming**: If Qdrant's API continues to evolve rapidly, this specific method (`query_points`) or its parameter names (`vector`) might change again, necessitating future code modifications.

## Alternatives Considered

*   **Alternative 1: Upgrade `qdrant-client` Library**:
    *   **Description**: Attempt to upgrade the `qdrant-client` library to the latest stable version (beyond 1.16.2), with the hope that a newer version provides a more consistent, clearly documented, or simplified API for vector search.
    *   **Pros**: May resolve existing API inconsistencies, potentially offers new features or performance improvements, aligns with latest best practices from Qdrant.
    *   **Cons**: Could introduce breaking changes in other parts of the ingestion pipeline (Spec-1) that currently rely on `qdrant-client==1.16.2`, requires thorough regression testing across both ingestion and retrieval, might still face similar API ambiguities or new ones.
*   **Alternative 2: Wrap `QdrantClient` with Custom Abstraction Layer**:
    *   **Description**: Create a thin wrapper class or set of helper functions around the `QdrantClient` instance to provide a stable, project-specific `search_by_vector` method that abstracts away the underlying `qdrant-client` method calls (`query`, `query_points`, or `search`).
    *   **Pros**: Provides a stable internal API for the project, isolates future `qdrant-client` changes from core business logic, improves readability and maintainability for project-specific search operations.
    *   **Cons**: Adds a layer of abstraction that requires initial development effort for the wrapper, still needs to correctly implement the underlying `qdrant-client` call within the wrapper.

## References

- Feature Spec: D:\CODING\physicalAI-Humanoid-robotics\specs\004-rag-retrieval-validation\spec.md
- Implementation Plan: D:\CODING\physicalAI-Humanoid-robotics\specs\004-rag-retrieval-validation\plan.md
- Research: D:\CODING\physicalAI-Humanoid-robotics\specs\004-rag-retrieval-validation\research.md (R-001)
- Related ADRs: None
- Evaluator Evidence: None
