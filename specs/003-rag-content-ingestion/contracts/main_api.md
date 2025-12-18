# Internal API Contracts: Main Pipeline Functions

**Feature**: 003-rag-content-ingestion
**Date**: 2025-12-18
**Context**: This document defines the internal function signatures and their responsibilities within the `main.py` script for the RAG content ingestion pipeline. These serve as the modular interfaces for the pipeline components.

## Function Signatures

### `get_all_urls`

-   **Signature**: `def get_all_urls(base_url: str) -> List[str]:`
-   **Description**: Discovers all public documentation URLs from the specified Docusaurus `base_url`. This typically involves parsing the `sitemap.xml` or recursively following internal links.
-   **Input**:
    -   `base_url` (str): The base URL of the deployed Docusaurus website (e.g., `https://physical-ai-humanoid-robotics-umber.vercel.app/`).
-   **Output**:
    -   `List[str]`: A list of absolute URLs for all discoverable pages.
-   **Errors**:
    -   `requests.exceptions.RequestException`: For network-related issues when fetching sitemap or pages.
    -   `ValueError`: If the sitemap cannot be found or parsed correctly.

### `extract_text_from_url`

-   **Signature**: `def extract_text_from_url(url: str) -> Tuple[str, Dict[str, Any]]:`
-   **Description**: Fetches the HTML content of a given Docusaurus page and extracts clean, structured text, along with relevant page-level metadata.
-   **Input**:
    -   `url` (str): The absolute URL of the Docusaurus page to process.
-   **Output**:
    -   `Tuple[str, Dict[str, Any]]`: A tuple containing:
        -   `str`: The clean, extracted text content of the page.
        -   `Dict[str, Any]`: A dictionary of page metadata (e.g., `{'title': 'Page Title', 'document_section': '...', 'last_modified': '...'}`).
-   **Errors**:
    -   `requests.exceptions.RequestException`: For issues fetching the page content.
    -   `BeautifulSoup.errors.MarkupResemblesLocatorError` (or similar parsing error): If HTML parsing fails unexpectedly.

### `chunk_text`

-   **Signature**: `def chunk_text(text: str, page_metadata: Dict[str, Any]) -> List[Dict[str, Any]]:`
-   **Description**: Applies a semantic chunking strategy to the extracted text, ensuring each chunk is contextually rich and associates it with relevant metadata.
-   **Input**:
    -   `text` (str): The full extracted text content from `extract_text_from_url`.
    -   `page_metadata` (Dict[str, Any]): Page-level metadata obtained from `extract_text_from_url`.
-   **Output**:
    -   `List[Dict[str, Any]]`: A list of dictionaries, where each dictionary represents a `TextChunk` and includes its text and initial metadata fields (e.g., `{'text': '...', 'source_url': '...', 'document_section': '...', 'heading': '...'}`). A unique `chunk_id` will be generated and added here.
-   **Errors**:
    -   `ValueError`: If the input `text` is empty or chunking fails for a specific reason.

### `embed_chunks`

-   **Signature**: `def embed_chunks(chunks: List[Dict[str, Any]], cohere_api_key: str) -> List[Tuple[VectorEmbedding, EmbeddingMetadata]]:`
-   **Description**: Generates vector embeddings for a list of processed text chunks using the Cohere API.
-   **Input**:
    -   `chunks` (List[Dict[str, Any]]): A list of chunk dictionaries, as produced by `chunk_text`.
    -   `cohere_api_key` (str): The API key for accessing the Cohere embedding service.
-   **Output**:
    -   `List[Tuple[VectorEmbedding, EmbeddingMetadata]]`: A list of tuples, each containing:
        -   `VectorEmbedding`: An object or named tuple representing the vector itself (`vector`, `dimensionality`, `model_name`).
        -   `EmbeddingMetadata`: An object or named tuple containing all metadata for Qdrant payload, including the `chunk_id`.
-   **Errors**:
    -   `cohere.APIError`: For any errors returned by the Cohere API (e.g., invalid API key, rate limits).
    -   `ValueError`: If the input `chunks` list is empty or malformed.

### `create_qdrant_collection`

-   **Signature**: `def create_qdrant_collection(qdrant_client: Any, collection_name: str, vector_size: int, distance: str = "Cosine") -> None:`
-   **Description**: Initializes a Qdrant collection with the specified name, vector size, and distance metric if it does not already exist.
-   **Input**:
    -   `qdrant_client` (Any): An initialized Qdrant client instance.
    -   `collection_name` (str): The name of the collection to create (e.g., `rag_embedding`).
    -   `vector_size` (int): The dimensionality of the vectors to be stored in the collection.
    -   `distance` (str): The distance metric for the collection (defaults to "Cosine").
-   **Output**:
    -   `None`.
-   **Errors**:
    -   `QdrantClientException`: For issues communicating with Qdrant or creating the collection.

### `save_chunks_to_qdrant`

-   **Signature**: `def save_chunks_to_qdrant(qdrant_client: Any, collection_name: str, embedded_chunks: List[Tuple[VectorEmbedding, EmbeddingMetadata]]) -> None:`
-   **Description**: Stores a list of embedded chunks (vectors and their associated metadata) into the specified Qdrant collection.
-   **Input**:
    -   `qdrant_client` (Any): An initialized Qdrant client instance.
    -   `collection_name` (str): The name of the target Qdrant collection.
    -   `embedded_chunks` (List[Tuple[VectorEmbedding, EmbeddingMetadata]]): A list of tuples, each containing a `VectorEmbedding` and its `EmbeddingMetadata`.
-   **Output**:
    -   `None`.
-   **Errors**:
    -   `QdrantClientException`: For issues during data insertion into Qdrant.
    -   `ValueError`: If `embedded_chunks` is empty or contains malformed data.

### `validate_ingestion`

-   **Signature**: `def validate_ingestion(qdrant_client: Any, collection_name: str, test_query: str, expected_results: List[str], cohere_api_key: str) -> bool:`
-   **Description**: Performs a similarity search in the Qdrant collection using a test query and validates the relevance of the retrieved results against a predefined set of expected outcomes.
-   **Input**:
    -   `qdrant_client` (Any): An initialized Qdrant client instance.
    -   `collection_name` (str): The name of the Qdrant collection to query.
    -   `test_query` (str): The query string to embed and use for similarity search.
    -   `expected_results` (List[str]): A list of expected identifiers (e.g., `chunk_id`s or `source_url`s) that should be highly ranked.
    -   `cohere_api_key` (str): Cohere API key to embed the `test_query`.
-   **Output**:
    -   `bool`: `True` if the validation criteria (e.g., expected results are within top N) are met, `False` otherwise.
-   **Errors**:
    -   `QdrantClientException`: For issues during similarity search.
    -   `cohere.APIError`: For issues embedding the test query.

### `main`

-   **Signature**: `def main() -> None:`
-   **Description**: The entry point for the entire data ingestion pipeline. It orchestrates the sequence of operations: URL collection, text extraction, chunking, embedding, Qdrant collection creation, and data storage.
-   **Input**:
    -   None (relies on environment variables or a configuration file for API keys, URLs, etc.).
-   **Output**:
    -   `None` (prints execution status, logs errors, and indicates completion).
-   **Errors**:
    -   Catches and logs exceptions from any of the pipeline steps, indicating specific failure points.