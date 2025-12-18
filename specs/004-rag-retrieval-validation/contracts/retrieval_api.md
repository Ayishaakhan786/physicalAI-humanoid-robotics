# Internal API Contracts: Retrieval Pipeline Functions

**Feature**: 004-rag-retrieval-validation
**Date**: 2025-12-18
**Context**: This document defines the internal function signatures and their responsibilities for the RAG retrieval and validation pipeline. These functions will likely reside in new modules or extend existing ones (e.g., `embedding.py`, `vector_db.py`, `validation.py`).

## Function Signatures

### `embed_query`

-   **Signature**: `def embed_query(query: str, cohere_api_key: str) -> List[float]:`
-   **Description**: Generates a vector embedding for a single query string using the Cohere API. This function already exists in `validation.py` but may be refactored to a more general `embedding.py` if other parts of the system also need query embedding.
-   **Input**:
    -   `query` (str): The natural language query to embed.
    -   `cohere_api_key` (str): The API key for accessing the Cohere embedding service.
-   **Output**:
    -   `List[float]`: The numerical vector embedding of the query.
-   **Errors**:
    -   `cohere.APIError`: For any errors returned by the Cohere API (e.g., invalid API key, rate limits).

### `retrieve_chunks`

-   **Signature**: `def retrieve_chunks(qdrant_client: QdrantClient, collection_name: str, query_vector: List[float], limit: int = 5, min_score: float = 0.0, filters: Optional[models.Filter] = None) -> List[RetrievedResult]:`
-   **Description**: Performs a semantic similarity search in the specified Qdrant collection using a pre-embedded query vector and returns ranked retrieved chunks with their metadata. This function will likely be an extension to `vector_db.py`.
-   **Input**:
    -   `qdrant_client` (QdrantClient): An initialized Qdrant client instance.
    -   `collection_name` (str): The name of the Qdrant collection to query (`rag_embedding`).
    -   `query_vector` (List[float]): The pre-embedded query vector.
    -   `limit` (int): The maximum number of results to return (default: 5).
    -   `min_score` (float): Minimum relevance score for a result to be included (default: 0.0).
    -   `filters` (Optional[models.Filter]): Qdrant filter object to apply metadata filtering.
-   **Output**:
    -   `List[RetrievedResult]`: A list of `RetrievedResult` objects, ranked by score.
-   **Errors**:
    -   `qdrant_client.http.exceptions.UnexpectedResponse` (or similar): For Qdrant API errors.

### `evaluate_retrieval`

-   **Signature**: `def evaluate_retrieval(retrieved_results: List[RetrievedResult], test_case: RetrievalTestCase) -> Dict[str, Any]:`
-   **Description**: Evaluates the relevance and correctness of retrieved results against a defined `RetrievalTestCase`. This function will likely reside in `validation.py`.
-   **Input**:
    -   `retrieved_results` (List[RetrievedResult]): The results obtained from `retrieve_chunks`.
    -   `test_case` (RetrievalTestCase): The test case containing expected outcomes.
-   **Output**:
    -   `Dict[str, Any]`: A dictionary containing evaluation metrics (e.g., `{'passed_expected_chunks': True, 'score_threshold_met': True, 'latency_ms': 120}`).
-   **Errors**:
    -   `ValueError`: If `retrieved_results` or `test_case` is malformed.

### `run_retrieval_test_suite`

-   **Signature**: `def run_retrieval_test_suite(test_cases: List[RetrievalTestCase], qdrant_client: QdrantClient, cohere_api_key: str, collection_name: str) -> Dict[str, Any]:`
-   **Description**: Orchestrates the execution of a suite of retrieval test cases, embedding queries, performing retrieval, and evaluating results. This function will be the main entry point for running the validation locally.
-   **Input**:
    -   `test_cases` (List[RetrievalTestCase]): A list of test cases to execute.
    -   `qdrant_client` (QdrantClient): An initialized Qdrant client instance.
    -   `cohere_api_key` (str): Cohere API key for query embedding.
    -   `collection_name` (str): The name of the Qdrant collection.
-   **Output**:
    -   `Dict[str, Any]`: A summary of overall test results (e.g., `{'total_tests': 10, 'passed': 8, 'failed': 2, 'details': [...]}`).
-   **Errors**:
    -   Propagates errors from `embed_query` or `retrieve_chunks`.

### `main` (Entry Point for Retrieval Tests)

-   **Signature**: `def main() -> None:`
-   **Description**: The entry point for running the local retrieval test suite. It will load configuration, prepare test cases, initialize clients, and call `run_retrieval_test_suite`.
-   **Input**:
    -   None (configuration from environment variables/config file).
-   **Output**:
    -   `None` (prints/logs test results).
-   **Errors**:
    -   Catches and logs exceptions, indicating failure points.
