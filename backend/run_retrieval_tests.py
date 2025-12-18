# backend/run_retrieval_tests.py
import logging
import time
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient, models
from cohere import Client

from src.utils.config import Config
from src.embedding import embed_query # Assuming embed_query is now in embedding.py
from src.models import RetrievedResult, RetrievalTestCase # Assuming these are defined in models.py
from src.retrieval_service import retrieve_chunks, evaluate_retrieval # Assuming retrieve_chunks and evaluate_retrieval are here
from src.utils.test_data_loader import load_retrieval_test_cases # Assuming this utility exists

# Configure basic logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def run_retrieval_test_suite(
    test_cases: List[RetrievalTestCase],
    qdrant_client: QdrantClient,
    cohere_api_key: str,
    collection_name: str
) -> Dict[str, Any]:
    """
    Orchestrates the execution of a suite of retrieval test cases,
    embedding queries, performing retrieval, and evaluating results.
    """
    logging.info(f"--- Running Retrieval Test Suite ({len(test_cases)} test cases) ---")
    
    total_tests = len(test_cases)
    passed_tests = 0
    test_results = []
    
    all_embed_latencies = []
    all_retrieve_latencies = []

    for i, test_case in enumerate(test_cases):
        logging.info(f"\nTest Case {i+1}/{total_tests}: Query='{test_case.query}' (Out-of-scope: {test_case.is_out_of_scope})")
        try:
            # Embed the query
            start_embed_time = time.perf_counter()
            query_vector = embed_query(test_case.query, cohere_api_key)
            embed_latency_ms = (time.perf_counter() - start_embed_time) * 1000
            all_embed_latencies.append(embed_latency_ms)
            logging.info(f"Query embedded (latency: {embed_latency_ms:.2f} ms).")

            # Perform retrieval
            start_retrieve_time = time.perf_counter()
            retrieved_results = retrieve_chunks(
                qdrant_client=qdrant_client,
                collection_name=collection_name,
                query_vector=query_vector,
                limit=5, # Limit to top 5 results for validation as per spec SC-001
                min_score=test_case.min_score_threshold if test_case.is_out_of_scope else 0.0,
            )
            retrieve_latency_ms = (time.perf_counter() - start_retrieve_time) * 1000
            all_retrieve_latencies.append(retrieve_latency_ms)
            logging.info(f"Retrieval performed (latency: {retrieve_latency_ms:.2f} ms). Found {len(retrieved_results)} results.")

            # Evaluate results
            evaluation_output = evaluate_retrieval(retrieved_results, test_case)
            
            test_passed = evaluation_output.get("overall_passed", False)

            if test_passed:
                passed_tests += 1
                logging.info("Test PASSED.")
            else:
                logging.warning(f"Test FAILED. Details: {evaluation_output.get('details')}")
            
            test_results.append({
                "test_case_query": test_case.query,
                "passed": test_passed,
                "embed_latency_ms": embed_latency_ms,
                "retrieve_latency_ms": retrieve_latency_ms,
                "evaluation_output": evaluation_output,
                "retrieved_results": [r.model_dump() for r in retrieved_results]
            })

        except Exception as e:
            logging.error(f"Error running test case for query '{test_case.query}': {e}")
            test_results.append({
                "test_case_query": test_case.query,
                "passed": False,
                "error": str(e)
            })
            
    avg_embed_latency = sum(all_embed_latencies) / len(all_embed_latencies) if all_embed_latencies else 0
    avg_retrieve_latency = sum(all_retrieve_latencies) / len(all_retrieve_latencies) if all_retrieve_latencies else 0

    overall_summary = {
        "total_tests": total_tests,
        "passed_tests": passed_tests,
        "failed_tests": total_tests - passed_tests,
        "pass_rate": (passed_tests / total_tests) * 100 if total_tests > 0 else 0,
        "average_embed_latency_ms": avg_embed_latency,
        "average_retrieve_latency_ms": avg_retrieve_latency,
        "test_results": test_results
    }
    
    logging.info("\n--- Retrieval Test Suite Finished ---")
    logging.info(f"Overall: Passed {passed_tests}/{total_tests} tests ({overall_summary['pass_rate']:.2f}%).")
    logging.info(f"Average Embedding Latency: {avg_embed_latency:.2f} ms")
    logging.info(f"Average Retrieval Latency: {avg_retrieve_latency:.2f} ms")
    
    return overall_summary

if __name__ == "__main__":
    try:
        Config.validate()
    except ValueError as e:
        logging.error(f"Configuration Error: {e}")
        exit(1)

    cohere_api_key = Config.COHERE_API_KEY
    qdrant_url = Config.QDRANT_URL
    qdrant_api_key = Config.QDRANT_API_KEY
    collection_name = "rag_embedding"
    test_case_file_path = "backend/data/retrieval_test_cases.json"

    # Initialize Qdrant Client
    qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

    try:
        logging.info(f"Loading test cases from {test_case_file_path}...")
        test_cases_to_run = load_retrieval_test_cases(test_case_file_path)
        logging.info(f"Loaded {len(test_cases_to_run)} test cases.")
    except FileNotFoundError as e:
        logging.error(f"Error loading test cases: {e}")
        exit(1)
    except Exception as e:
        logging.error(f"An unexpected error occurred while loading test cases: {e}")
        exit(1)
    
    overall_results = run_retrieval_test_suite(
        test_cases=test_cases_to_run,
        qdrant_client=qdrant_client,
        cohere_api_key=cohere_api_key,
        collection_name=collection_name
    )
    logging.info(f"Final Test Summary: {overall_results['pass_rate']:.2f}% passed.")
    # You can further process overall_results here, e.g., save to a report file
    # print(json.dumps(overall_results, indent=2))
