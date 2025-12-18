# backend/src/retrieval_service.py
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient, models
from src.models import RetrievedResult, EmbeddingMetadata
from src.utils.decorators import retry
from requests import RequestException
from qdrant_client.http.exceptions import UnexpectedResponse, ResponseHandlingException

@retry(exceptions=(RequestException, UnexpectedResponse, ResponseHandlingException), tries=5)
def retrieve_chunks(
    qdrant_client: QdrantClient,
    collection_name: str,
    query_vector: List[float],
    limit: int = 5,
    min_score: float = 0.0,
    filters: Optional[models.Filter] = None
) -> List[RetrievedResult]:
    """
    Performs a semantic similarity search in the specified Qdrant collection
    using a pre-embedded query vector and returns ranked retrieved chunks with their metadata.
    """
    if not query_vector:
        return []

    try:
        search_result = qdrant_client.query_points(
            collection_name=collection_name,
            query_filter=filters, # Apply filters if provided
            vector=query_vector,
            limit=limit,
            score_threshold=min_score,
            with_payload=True # Always retrieve payload for metadata
        )

        retrieved_results: List[RetrievedResult] = []
        for hit in search_result:
            # Assuming payload is directly compatible with EmbeddingMetadata or can be constructed
            if hit.payload:
                embed_meta = EmbeddingMetadata(**hit.payload)
                retrieved_results.append(
                    RetrievedResult(
                        chunk_id=embed_meta.chunk_id,
                        score=hit.score,
                        retrieved_chunk_metadata=embed_meta,
                        retrieved_chunk_text=embed_meta.chunk_text_preview # Or a full text field if available
                    )
                )
        return retrieved_results
    except Exception as e:
        print(f"Error during Qdrant retrieval: {e}")
        raise

def evaluate_retrieval(retrieved_results: List[RetrievedResult], test_case: RetrievalTestCase) -> Dict[str, Any]:
    """
    Evaluates the relevance and correctness of retrieved results against a defined test case.
    Assesses relevance, metadata integrity, and out-of-scope handling.
    """
    evaluation_results = {
        "query": test_case.query,
        "passed_relevance": False,
        "passed_metadata_integrity": True, # Assume true unless a check fails
        "passed_out_of_scope_check": False,
        "details": [],
        "num_retrieved_results": len(retrieved_results),
        "top_scores": [r.score for r in retrieved_results[:3]] # Top 3 scores
    }

    # Check Metadata Integrity for all retrieved results
    for i, res in enumerate(retrieved_results):
        if not res.retrieved_chunk_metadata or not res.retrieved_chunk_metadata.source_url or not res.retrieved_chunk_metadata.chunk_id:
            evaluation_results["passed_metadata_integrity"] = False
            evaluation_results["details"].append(f"Metadata missing for retrieved result {i+1}")
            break
    if not evaluation_results["passed_metadata_integrity"]:
        evaluation_results["details"].append("Metadata integrity check failed.")

    if test_case.is_out_of_scope:
        # For out-of-scope queries, we expect no relevant results or very low scores
        # Check if no results were returned or all results are below threshold
        if len(retrieved_results) == 0:
            evaluation_results["passed_out_of_scope_check"] = True
            evaluation_results["passed_relevance"] = True # No relevant results is desired
            evaluation_results["details"].append("Out-of-scope query returned no results (desired).")
        elif all(r.score < test_case.min_score_threshold for r in retrieved_results):
            evaluation_results["passed_out_of_scope_check"] = True
            evaluation_results["passed_relevance"] = True # Low scores are desired
            evaluation_results["details"].append(f"Out-of-scope query results all below score threshold {test_case.min_score_threshold} (desired).")
        else:
            evaluation_results["passed_out_of_scope_check"] = False
            evaluation_results["passed_relevance"] = False # Found relevant results for out-of-scope
            evaluation_results["details"].append("Out-of-scope query returned relevant results (undesired).")
    else:
        # For in-scope queries, check for expected chunks/fragments
        relevance_score = 0
        if test_case.expected_chunk_ids:
            retrieved_chunk_ids = {r.chunk_id for r in retrieved_results}
            # Count how many expected chunk IDs are in the top results
            matched_ids = len([cid for cid in test_case.expected_chunk_ids if cid in retrieved_chunk_ids])
            if matched_ids > 0:
                relevance_score += 0.5 # Partial pass for IDs
            evaluation_results["details"].append(f"Matched {matched_ids}/{len(test_case.expected_chunk_ids)} expected chunk IDs.")

        if test_case.expected_fragments:
            found_fragments_count = 0
            for frag in test_case.expected_fragments:
                # Check if fragment is in any of the top retrieved texts
                if any(frag.lower() in r.retrieved_chunk_text.lower() for r in retrieved_results):
                    found_fragments_count += 1
            if found_fragments_count > 0:
                relevance_score += 0.5 # Partial pass for fragments
            evaluation_results["details"].append(f"Matched {found_fragments_count}/{len(test_case.expected_fragments)} expected fragments.")
        
        evaluation_results["passed_relevance"] = (relevance_score >= 0.5) # At least partial match
        if not evaluation_results["passed_relevance"]:
             evaluation_results["details"].append("Relevance check failed: No expected chunk IDs or fragments found.")

    # Overall pass based on all checks
    overall_passed = evaluation_results["passed_relevance"] and \
                     evaluation_results["passed_metadata_integrity"] and \
                     (evaluation_results["passed_out_of_scope_check"] if test_case.is_out_of_scope else True) # Only check if out of scope
    
    evaluation_results["overall_passed"] = overall_passed
    return evaluation_results