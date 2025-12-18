# ADR-0006: RAG Retrieval Pipeline Core Technology Stack

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

-   **Status:** Proposed
-   **Date:** 2025-12-18
-   **Feature:** 004-rag-retrieval-validation
-   **Context:** The RAG Retrieval Pipeline requires selecting core technologies for query embedding and semantic similarity search. This decision formalizes the choice of specific services and libraries, ensuring consistency with the ingestion pipeline (Spec-1) and defining the fundamental components that will dictate retrieval performance, cost, and developer experience. This selection is crucial for the accuracy and efficiency of the entire RAG system.

## Decision

The RAG retrieval pipeline will utilize Cohere's `embed-english-v3.0` model for generating embeddings from natural language queries. For semantic similarity search and storage, the existing `rag_embedding` collection within Qdrant Cloud will be used, continuing the technology choices established in Spec-1 for content ingestion. Python will remain the primary implementation language, leveraging its rich ecosystem for client libraries to interact with these services.

## Consequences

### Positive

*   **Consistency with Ingestion**: Reusing the same Cohere embedding model and Qdrant collection as the ingestion pipeline (Spec-1) ensures full compatibility of query embeddings with stored document embeddings, which is critical for accurate similarity matching.
*   **Leverages Existing Infrastructure**: Building upon the already established Qdrant Cloud instance and Cohere API integration reduces setup overhead and accelerates development for the retrieval component.
*   **High-Quality Embeddings**: Continues to benefit from Cohere's robust and high-quality embedding capabilities, leading to potentially more accurate semantic searches.
*   **Managed Vector Database**: Qdrant Cloud handles the underlying infrastructure, scaling, and maintenance of the vector store, freeing up engineering resources.

### Negative

*   **Vendor Lock-in**: Continued reliance on Cohere and Qdrant Cloud introduces vendor lock-in for both embedding generation and vector database functionality within the retrieval phase.
*   **Cost Implications**: Scaling retrieval queries will directly incur costs for Cohere API usage (for query embedding) and Qdrant Cloud services, which need to be monitored, especially if moving beyond Free Tier limits.
*   **Performance Dependency**: The end-to-end retrieval performance (latency and throughput) is heavily tied to the responsiveness of the Cohere API for query embedding and Qdrant's indexing and querying capabilities.
*   **Single Point of Failure**: A strong dependency on external Cohere and Qdrant services introduces potential points of failure if these services experience outages or performance degradation.

## Alternatives Considered

*   **Alternative 1: Alternative Embedding Provider + Vector DB**:
    *   **Description**: Explore using different leading providers for query embedding (e.g., OpenAI's models, Google's `text-embedding-004`) and alternative vector databases (e.g., Pinecone, Weaviate, Milvus).
    *   **Pros**: Diversifies technology choices, potentially finds a more cost-effective or performant solution for retrieval, reduces vendor lock-in to Cohere/Qdrant.
    *   **Cons**: Requires re-embedding existing data in Qdrant if the new embedding model is incompatible, significant development effort for new API integrations, increased complexity due to managing different services, potentially higher learning curve for new platforms.
*   **Alternative 2: Local Embedding Model for Queries**:
    *   **Description**: Investigate and integrate a smaller, open-source, local embedding model for query embedding (e.g., sentence-transformers, FastEmbed) to reduce reliance on external API calls and latency.
    *   **Pros**: Reduces external API calls and associated costs, potentially lower latency for query embedding, greater control over the embedding process.
    *   **Cons**: Requires managing local model inference (CPU/GPU resources), local model must be rigorously tested for compatibility with existing Qdrant embeddings from Cohere, potentially lower embedding quality compared to large cloud models.

## References

-   Feature Spec: D:\CODING\physicalAI-Humanoid-robotics\specs\004-rag-retrieval-validation\spec.md
-   Implementation Plan: D:\CODING\physicalAI-Humanoid-robotics\specs\004-rag-retrieval-validation\plan.md
-   Related ADRs: 0003-rag-pipeline-technology-stack-selection.md (Ingestion Stack)
-   Evaluator Evidence: None
