# Research Findings: RAG Content Ingestion and Vector Storage

**Feature**: 003-rag-content-ingestion
**Date**: 2025-12-18

## Overview of Research Tasks

The following research tasks were identified to address technical unknowns and inform detailed design decisions for the RAG content ingestion and vector storage pipeline.

## Findings

### R-001: Efficient Docusaurus Content Crawling Strategies

-   **Objective**: Determine the most effective way to discover all public URLs from the Docusaurus book (`https://physical-ai-humanoid-robotics-umber.vercel.app/`) and extract clean, structured textual content from its HTML.
-   **Decision**: [NEEDS RESEARCH: Crawling strategy (e.g., sitemap parsing, recursive linking) and HTML parsing approach (e.g., BeautifulSoup selectors for content div).]
-   **Rationale**: [NEEDS RESEARCH: Justification for chosen strategy, considering robustness, efficiency, and content fidelity.]
-   **Alternatives Considered**: [NEEDS RESEARCH: Other crawling tools/libraries, different HTML parsing techniques.]

### R-002: Cohere Embedding Model Specifics

-   **Objective**: Identify the latest stable Cohere embedding model and understand its integration requirements and characteristics.
-   **Decision**: [NEEDS RESEARCH: Specific Cohere model name (e.g., `embed-english-v3.0`), exact API client usage, input/output limits, dimensionality.]
-   **Rationale**: [NEEDS RESEARCH: Justification for model choice, considering quality, performance, and cost.]
-   **Alternatives Considered**: [NEEDS RESEARCH: Older Cohere models, other embedding providers (e.g., OpenAI, Google).]

### R-003: Qdrant Cloud Free Tier and Python Client Usage

-   **Objective**: Document Qdrant Cloud Free Tier limitations and understand the Qdrant Python client for collection management, data insertion, and querying.
-   **Decision**: [NEEDS RESEARCH: Detailed Free Tier limits (storage, RPS), Python client connection details, collection creation with specific config, point insertion with payload, similarity search examples.]
-   **Rationale**: [NEEDS RESEARCH: How Qdrant client methods map to pipeline requirements.]
-   **Alternatives Considered**: [NEEDS RESEARCH: Local Qdrant instance, other vector databases (e.g., Pinecone, Weaviate).]

### R-004: Semantic Text Chunking Libraries

-   **Objective**: Evaluate Python libraries for semantic text chunking, particularly for technical documentation with mixed content.
-   **Decision**: [NEEDS RESEARCH: Specific chunking library/method (e.g., `RecursiveCharacterTextSplitter`), optimal parameters (chunk size, overlap) for Docusaurus content.]
-   **Rationale**: [NEEDS RESEARCH: Why the chosen library/method best preserves semantic meaning and handles code blocks/headings.]
-   **Alternatives Considered**: [NEEDS RESEARCH: Other chunking strategies (e.g., token-based, paragraph-based), custom chunking logic.]

### R-005: `uv` for Python Project Management

-   **Objective**: Research best practices for initializing a Python project with `uv` for environment and dependency management.
-   **Decision**: [NEEDS RESEARCH: Recommended `uv` commands for virtual environment creation, dependency installation (from `requirements.txt` or `pyproject.toml`), and project execution.]
-   **Rationale**: [NEEDS RESEARCH: How `uv` streamlines development and ensures reproducible builds.]
-   **Alternatives Considered**: [NEEDS RESEARCH: `pipenv`, `poetry`, `conda`, `venv` + `pip`.]

---
