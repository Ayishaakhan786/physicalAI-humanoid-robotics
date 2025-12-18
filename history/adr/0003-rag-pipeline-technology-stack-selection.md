# ADR-0003: RAG Pipeline Technology Stack Selection

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

-   **Status:** Proposed
-   **Date:** 2025-12-18
-   **Feature:** 003-rag-content-ingestion
-   **Context:** The core functionality of the RAG content ingestion pipeline relies on specific external services and libraries for embedding generation, vector storage, and content acquisition. The initial feature specification outlines key providers and technologies. This decision cluster formalizes the selection of these core components which will heavily influence the pipeline's performance, cost, and long-term maintainability.

## Decision

The RAG pipeline will leverage Cohere for generating high-quality text embeddings, utilizing the latest stable embedding model. Qdrant Cloud (specifically its Free Tier) is selected as the vector database for storing and indexing these embeddings. Content acquisition from the deployed Docusaurus website will be handled by standard Python web scraping libraries, with `requests` for fetching HTML content and `BeautifulSoup` for parsing. A dedicated Python library will be used for semantic text chunking to ensure proper context preservation.

## Consequences

### Positive

*   **Adherence to Constraints**: Directly fulfills the specified project constraints for using Cohere as the embedding provider and Qdrant Cloud (Free Tier) as the vector database.
*   **Leverages Managed Services**: Qdrant Cloud significantly reduces the operational overhead associated with setting up, maintaining, and scaling a vector database.
*   **High-Quality Embeddings**: Cohere models are recognized for generating robust and semantically rich embeddings, which is crucial for the effectiveness of a RAG system.
*   **Proven Web Scraping**: The combination of `requests` and `BeautifulSoup` offers a widely adopted, flexible, and powerful solution for web content extraction in Python.
*   **Semantic Chunking**: The use of a specialized library for semantic chunking ensures that textual context is appropriately preserved, leading to more meaningful embeddings and better retrieval performance.

### Negative

*   **Vendor Lock-in**: Reliance on specific third-party services like Cohere and Qdrant Cloud introduces a degree of vendor lock-in, which could complicate switching providers in the future or impact cost flexibility.
*   **Free Tier Limitations**: The Qdrant Cloud Free Tier might impose limitations on storage capacity, request rates, or available features that could become restrictive as the Docusaurus book grows or ingestion frequency increases, potentially necessitating an upgrade.
*   **Web Scraping Fragility**: Web scraping is inherently fragile; changes to the Docusaurus website's HTML structure could break the content extraction logic, requiring frequent maintenance.
*   **Cost Implications**: While starting with Free Tier, scaling up (more content, higher query volume) will inevitably incur costs for Cohere API usage and Qdrant Cloud services.

## Alternatives Considered

*   **Alternative 1: OpenAI/Google Embeddings + Pinecone/Weaviate Vector DB**:
    *   **Description**: Use alternative leading providers for embeddings (e.g., OpenAI's `text-embedding-ada-002`, Google's `text-embedding-004`) and alternative managed vector databases (e.g., Pinecone, Weaviate Cloud).
    *   **Pros**: Access to different models with potentially different performance characteristics, broader ecosystem choice, competitive pricing.
    *   **Cons**: Requires re-evaluation of integration, potentially different API paradigms, may not align with existing team expertise or preferences.
*   **Alternative 2: Self-hosted Vector DB**:
    *   **Description**: Run a vector database (e.g., Qdrant, Milvus, Chroma) as a self-hosted instance on project infrastructure.
    *   **Pros**: Provides full control over infrastructure, potentially lower costs at very large scale (if operational burden is manageable), no Free Tier limitations.
    *   **Cons**: Significantly increased operational overhead (setup, maintenance, scaling, backups, security), requires dedicated infrastructure and expertise.
*   **Alternative 3: Direct Markdown Parsing (if source available)**:
    *   **Description**: If direct access to the raw Docusaurus Markdown source files is feasible (e.g., from a local git repository or build artifacts), parse Markdown directly instead of scraping the deployed HTML.
    *   **Pros**: More robust to website UI changes, potentially easier to extract clean text and semantic metadata, avoids network latency and fragility of web scraping.
    *   **Cons**: Requires access to the source code repository or build system (not just the deployed URL), may miss dynamic content rendered client-side, fundamentally changes the 'content source' assumption.

## References

-   Feature Spec: D:\CODING\physicalAI-Humanoid-robotics\specs\003-rag-content-ingestion\spec.md
-   Implementation Plan: D:\CODING\physicalAI-Humanoid-robotics\specs\003-rag-content-ingestion\plan.md
-   Related ADRs: None
-   Evaluator Evidence: None
