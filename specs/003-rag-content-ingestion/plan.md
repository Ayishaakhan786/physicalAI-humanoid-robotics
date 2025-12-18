# Implementation Plan: RAG Content Ingestion and Vector Storage

**Project Name**: RAG Chatbot - Spec-1: Website Content Ingestion, Embedding Generation, and Vector Storage
**Feature Branch**: `003-rag-content-ingestion`
**Created**: 2025-12-18
**Status**: Draft

## Problem Statement

AI engineers and backend developers require a reliable data ingestion pipeline to extract content from a deployed Docusaurus website, generate high-quality vector embeddings using Cohere models, and store/index these embeddings in Qdrant Cloud (Free Tier) for downstream retrieval within a Retrieval-Augmented Generation (RAG) system. The solution must be implemented in Python, be modular, and include comprehensive documentation and a runnable script.

## Proposed Solution

Develop a Python-based data ingestion pipeline that will:
1.  **Crawl and Extract**: Systematically collect all public URLs from a specified Docusaurus book (e.g., `https://physical-ai-humanoid-robotics-umber.vercel.app/`) and extract clean, structured textual content.
2.  **Semantic Chunking**: Apply a semantic chunking strategy to segment the extracted text, ensuring context is preserved within each chunk.
3.  **Embedding Generation**: Generate high-quality vector embeddings for each text chunk using the latest stable Cohere embedding model.
4.  **Vector Storage**: Initialize a dedicated Qdrant collection named `rag_embedding` in Qdrant Cloud (Free Tier) and store all embedded chunks along with their associated metadata (URL, section, heading, chunk ID).
5.  **Validation**: Implement functionality to validate the ingestion process by performing similarity searches on stored vectors.
The entire solution will be contained within a single `main.py` file, utilizing specific, well-defined functions.

---

## Technical Context

### Key Technologies & Services
-   **Language**: Python
-   **Embedding Provider**: Cohere (latest stable embedding model)
-   **Vector Database**: Qdrant Cloud (Free Tier)
-   **Content Source**: Deployed Docusaurus website (`https://physical-ai-humanoid-robotics-umber.vercel.app/`)
-   **Environment/Dependency Management**: `uv`

### Constraints
-   Implementation Language: Python
-   Embedding Provider: Cohere (latest stable embedding model)
-   Vector Database: Qdrant Cloud (Free Tier)
-   Content Source: Deployed Docusaurus website URLs
-   Chunking Strategy: Must preserve semantic meaning
-   Modularity: Must be modular and reusable for future updates
-   Timeline: Complete within 3–5 days
-   Output Format: Markdown documentation + runnable script
-   Code Structure: Single `main.py` file with specific functions (`get_all_urls`, `extract_text_from_url`, `chunk_text`, `embed`, `create_collection` named `rag_embedding`, `save_chunk_to_qdrant`, `main`).

### Assumptions
-   Qdrant Cloud Free Tier limitations (e.g., storage capacity, request rate) are sufficient for the initial scope of the Docusaurus book. If the book grows significantly, a paid tier might be required.
-   Cohere API access and a valid API key will be provisioned prior to development.
-   Qdrant Cloud access and a valid API key (if required for Free Tier) will be provisioned.
-   The deployed Docusaurus website provides a standard sitemap (e.g., `sitemap.xml`) and a consistent HTML structure that allows for programmatic extraction of main textual content, headings, and URLs.
-   The `uv` tool is installed and available in the development environment.
-   Standard Python libraries for web scraping (e.g., `requests`, `BeautifulSoup`) are permissible for content extraction.
-   A suitable Python library for semantic text chunking (e.g., a component from `langchain` or `llama_index`) will be used.

---

## Constitution Check

The `constitution.md` file in `.specify/memory/` appears to be a template and does not define concrete project principles. Therefore, a full constitution check cannot be performed at this stage. It is assumed that once project-specific constitutional principles are established, this plan will be reviewed against them.

---

## Gates

### Initial Gate: Specification Review
-   **Status**: PASS
-   **Justification**: The feature specification (`spec.md`) has been reviewed. All requirements are clear, testable, and free of `[NEEDS CLARIFICATION]` markers. User stories are well-defined, and success criteria are measurable and technology-agnostic.

---

## Phases

### Phase 0: Outline & Research

**Goal**: To resolve all technical unknowns and gather necessary information for detailed design.

#### Research Tasks

-   **R-001**: Research efficient Docusaurus content crawling strategies:
    -   Focus: How to programmatically discover all public URLs from `https://physical-ai-humanoid-robotics-umber.vercel.app/` (e.g., `sitemap.xml` parsing).
    -   Focus: Best practices for extracting clean, structured textual content from Docusaurus-generated HTML, avoiding navigation, headers, footers, etc. (e.g., using `BeautifulSoup` to target content divs).
-   **R-002**: Identify Cohere embedding model specifics:
    -   Focus: Determine the latest stable Cohere embedding model suitable for general text.
    -   Focus: Understand its input requirements (max tokens, formatting) and output characteristics (dimensionality, data type).
    -   Focus: How to integrate the Cohere Python client for embedding generation.
-   **R-003**: Investigate Qdrant Cloud Free Tier and Python client usage:
    -   Focus: Document storage limits and any request rate limits for the Free Tier.
    -   Focus: Learn the Qdrant Python client for creating collections, defining schemas (including metadata fields), inserting vectors, and performing similarity searches.
    -   Focus: Understand how to connect to a Qdrant Cloud instance securely.
-   **R-004**: Evaluate Python libraries for semantic text chunking:
    -   Focus: Compare `langchain.text_splitter` and other options for their ability to preserve semantic meaning, especially with technical documentation (code blocks, headings).
    -   Focus: Determine optimal chunk size and overlap strategies for the Docusaurus content.
-   **R-005**: Research `uv` for Python project management:
    -   Focus: Best practices for initializing a Python project with `uv` (virtual environment creation, `pyproject.toml` or `requirements.txt`).
    -   Focus: How to manage dependencies, install packages, and ensure reproducibility.

#### Consolidated Findings (`research.md`)

*This section will be populated upon completion of the research tasks.*

---

### Phase 1: Design & Contracts

**Prerequisites**: All research tasks from Phase 0 are completed, and `research.md` is populated.

**Goal**: To define the data structures, internal function interfaces, and provide a quickstart guide for the implementation.

#### Data Model (`data-model.md`)

Based on the `Key Entities` identified in the feature specification, the following data model will guide the pipeline:

-   **DocusaurusPage**:
    -   `url` (string): The absolute URL of the Docusaurus page.
    -   `html_content` (string): Raw HTML content fetched from the URL.
    -   `extracted_text` (string): Clean, structured text extracted from `html_content`.
    -   `sitemap_last_modified` (datetime, optional): Timestamp from sitemap for change detection.
-   **TextChunk**:
    -   `id` (string): Unique identifier for the chunk (e.g., hash of content + metadata).
    -   `text` (string): The actual text content of the chunk.
    -   `length` (int): Number of tokens or characters in the chunk.
    -   `start_char_idx` (int, optional): Starting character index in the `extracted_text`.
    -   `end_char_idx` (int, optional): Ending character index in the `extracted_text`.
-   **VectorEmbedding**:
    -   `vector` (list of float): The numerical embedding generated by Cohere.
    -   `dimensionality` (int): The dimension of the vector.
    -   `model_name` (string): Name of the Cohere model used (e.g., `embed-english-v3.0`).
-   **EmbeddingMetadata**: (Corresponds to Qdrant Payload)
    -   `source_url` (string): URL of the original Docusaurus page.
    -   `document_section` (string, optional): E.g., 'module-01-ros2', 'intro'.
    -   `heading` (string, optional): The closest main heading (`<h1>` to `<h3>`) above the chunk.
    -   `chunk_id` (string): Unique identifier for the `TextChunk` (same as `TextChunk.id`).
    -   `chunk_text_preview` (string, optional): A short snippet of the chunk text for context.
    -   `updated_at` (datetime): Timestamp of when the embedding was last generated/stored.
-   **QdrantCollection**:
    -   `name` (string): `rag_embedding`.
    -   `vector_size` (int): Must match `VectorEmbedding.dimensionality`.
    -   `distance_metric` (string): E.g., `Cosine` (common for embeddings).
    -   `metadata_schema`: Implicitly defined by `EmbeddingMetadata` fields for filtering/payload.

#### Internal API Contracts (Function Signatures in `main.py`)

These functions represent the modular components of the pipeline within `main.py`.

-   `def get_all_urls(base_url: str) -> List[str]:`
    -   **Description**: Scrapes the Docusaurus sitemap to find all public documentation URLs.
    -   **Input**: `base_url` (e.g., `https://physical-ai-humanoid-robotics-umber.vercel.app/`).
    -   **Output**: A list of absolute URLs (`List[str]`).
    -   **Errors**: Raises `requests.exceptions.RequestException` for network issues, `ValueError` for sitemap parsing errors.

-   `def extract_text_from_url(url: str) -> Tuple[str, Dict[str, Any]]:`
    -   **Description**: Fetches the HTML content of a given Docusaurus page and extracts clean, structured text along with relevant metadata (e.g., title, main headings).
    -   **Input**: `url` of the Docusaurus page.
    -   **Output**: A tuple containing the `extracted_text` (str) and a dictionary of `page_metadata` (Dict[str, Any], e.g., `{'title': '...', 'sections': [...]}`).
    -   **Errors**: Raises `requests.exceptions.RequestException` for fetch errors, `BeautifulSoup.errors.MarkupResemblesLocatorError` for parsing issues.

-   `def chunk_text(text: str, page_metadata: Dict[str, Any]) -> List[Dict[str, Any]]:`
    -   **Description**: Applies semantic chunking to the extracted text, associating each chunk with relevant metadata (URL, section, heading).
    -   **Input**: `text` (str) - the full extracted text; `page_metadata` (Dict) - metadata from `extract_text_from_url`.
    -   **Output**: A list of dictionaries, where each dictionary represents a chunk and includes its text and initial metadata fields (`text`, `source_url`, `document_section`, `heading`).
    -   **Errors**: Raises `ValueError` if text is empty.

-   `def embed_chunks(chunks: List[Dict[str, Any]], cohere_api_key: str) -> List[Tuple[VectorEmbedding, EmbeddingMetadata]]:`
    -   **Description**: Generates vector embeddings for a list of text chunks using the Cohere API.
    -   **Input**: `chunks` (List[Dict[str, Any]]) - list of chunk dictionaries; `cohere_api_key` (str).
    -   **Output**: A list of tuples, where each tuple contains the `VectorEmbedding` (vector, dimensionality, model_name) and `EmbeddingMetadata` (all fields, including `chunk_id` generated here).
    -   **Errors**: Raises `cohere.APIError` for Cohere API issues, `ValueError` for empty chunks.

-   `def create_qdrant_collection(qdrant_client: Any, collection_name: str, vector_size: int) -> None:`
    -   **Description**: Creates a new Qdrant collection with a specified name and vector size if it doesn't already exist.
    -   **Input**: `qdrant_client` (QdrantClient instance); `collection_name` (str, e.g., `rag_embedding`); `vector_size` (int).
    -   **Output**: None.
    -   **Errors**: Raises `QdrantClientException` for Qdrant API issues.

-   `def save_chunks_to_qdrant(qdrant_client: Any, collection_name: str, embedded_chunks: List[Tuple[VectorEmbedding, EmbeddingMetadata]]) -> None:`
    -   **Description**: Stores a list of embedded chunks (vectors + metadata) into the specified Qdrant collection.
    -   **Input**: `qdrant_client` (QdrantClient instance); `collection_name` (str); `embedded_chunks` (List[Tuple[VectorEmbedding, EmbeddingMetadata]]).
    -   **Output**: None.
    -   **Errors**: Raises `QdrantClientException` for Qdrant API issues, `ValueError` for malformed input.

-   `def validate_ingestion(qdrant_client: Any, collection_name: str, test_query: str, expected_results: List[str]) -> bool:`
    -   **Description**: Performs a similarity search for a test query and validates the relevance of the retrieved results against expected outcomes.
    -   **Input**: `qdrant_client`, `collection_name`, `test_query` (str), `expected_results` (List[str], e.g., URLs or chunk text snippets).
    -   **Output**: `True` if validation passes, `False` otherwise.
    -   **Errors**: Raises `QdrantClientException` for search errors.

-   `def main() -> None:`
    -   **Description**: Orchestrates the entire pipeline, calling the above functions in sequence.
    -   **Input**: None (configuration from environment variables/config file).
    -   **Output**: None (prints status, logs errors).

#### Quickstart (`quickstart.md`)

*This will be generated in a separate step after the initial plan.md is complete.*

---

### Agent Context Update

*This will be done after `Phase 1` content is generated.*

---

I will now write this content to `specs/003-rag-content-ingestion/plan.md`.