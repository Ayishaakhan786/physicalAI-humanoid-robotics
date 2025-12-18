import logging
from typing import Any, List
from qdrant_client import QdrantClient
from src.embedding import embed_query

from src.utils.config import Config




def validate_ingestion(
    qdrant_client: QdrantClient,
    collection_name: str,
    test_query: str,
    expected_results_fragment: List[str], # List of text fragments expected in top results
    cohere_api_key: str
) -> bool:
    """
    Performs a similarity search for a test query and validates the relevance of the
    retrieved results against expected outcomes.
    
    Args:
        qdrant_client: An initialized Qdrant client instance.
        collection_name: The name of the Qdrant collection to query.
        test_query: The query string to embed and use for similarity search.
        expected_results_fragment: A list of text fragments that are expected
                                   to be present in the content of the top retrieved results.
        cohere_api_key: Cohere API key to embed the test_query.

    Returns:
        True if the validation criteria (e.g., expected results are within top N)
        are met, False otherwise.
    """
    logging.info(f"\n--- Running Validation for Query: '{test_query}' ---")
    
    # --- Diagnostic start ---
    logging.info(f"Diagnostic: Type of qdrant_client: {type(qdrant_client)}")
    logging.info(f"Diagnostic: dir(qdrant_client): {dir(qdrant_client)}")
    if hasattr(qdrant_client, 'search'):
        logging.info("Diagnostic: qdrant_client has 'search' method.")
    else:
        logging.info("Diagnostic: qdrant_client DOES NOT have 'search' method.")
    # --- Diagnostic end ---

    try:
        query_vector = embed_query(test_query, cohere_api_key)
        
        search_result = qdrant_client.query_points(
            collection_name=collection_name,
            vector=query_vector, # Changed from query_vector to vector
            limit=5, # Look at top 5 results
            with_payload=True # Retrieve payload to check text fragments
        )
        
        found_expected_fragments = [False] * len(expected_results_fragment)
        
        logging.info(f"Top {len(search_result)} search results:")
        for hit in search_result:
            payload_text_preview = hit.payload.get("chunk_text_preview", "")
            logging.info(f"  - Score: {hit.score:.2f}, URL: {hit.payload.get('source_url')}, Preview: {payload_text_preview[:100]}...")
            
            for i, expected_frag in enumerate(expected_results_fragment):
                if expected_frag.lower() in payload_text_preview.lower():
                    found_expected_fragments[i] = True
        
        validation_passed = all(found_expected_fragments)
        
        if validation_passed:
            logging.info("\nValidation PASSED: All expected fragments found in top results.")
        else:
            missing_fragments = [expected_results_fragment[i] for i, found in enumerate(found_expected_fragments) if not found]
            logging.warning(f"\nValidation FAILED: Missing expected fragments: {missing_fragments}")
            
        return validation_passed

    except Exception as e:
        logging.error(f"Error during validation: {e}")
        return False

# Example usage (requires Qdrant and Cohere setup)
if __name__ == "__main__":
    from qdrant_client import QdrantClient
    
    # Load config and validate
    try:
        Config.validate()
    except ValueError as e:
        logging.error(f"Configuration Error: {e}")
        exit(1)

    cohere_api_key = Config.COHERE_API_KEY
    qdrant_url = Config.QDRANT_URL
    qdrant_api_key = Config.QDRANT_API_KEY
    collection_name = "rag_embedding" # Must match collection in main.py

    # Initialize Qdrant Client
    qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

    # Example test query and expected results
    test_query_1 = "How to install Docusaurus?"
    expected_fragments_1 = ["install Node.js", "npx create-docusaurus"]

    test_query_2 = "What is a vector database?"
    expected_fragments_2 = ["Qdrant", "vector"] # Assuming your content has this

    # Run validation
    result1 = validate_ingestion(qdrant_client, collection_name, test_query_1, expected_fragments_1, cohere_api_key)
    logging.info(f"Result for query 1: {result1}")

    result2 = validate_ingestion(qdrant_client, collection_name, test_query_2, expected_fragments_2, cohere_api_key)
    logging.info(f"Result for query 2: {result2}")
