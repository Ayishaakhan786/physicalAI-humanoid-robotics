import json
from typing import List
from pathlib import Path
from src.models import RetrievalTestCase

def load_retrieval_test_cases(file_path: str) -> List[RetrievalTestCase]:
    """
    Loads retrieval test cases from a JSON file.
    
    The JSON file should be a list of dictionaries, where each dictionary
    conforms to the structure of RetrievalTestCase.
    """
    path = Path(file_path)
    if not path.exists():
        raise FileNotFoundError(f"Test case file not found: {file_path}")
    
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    # Validate each item against the Pydantic model
    test_cases = [RetrievalTestCase(**item) for item in data]
    return test_cases

# Example usage (requires backend/data/retrieval_test_cases.json)
if __name__ == "__main__":
    # Create a dummy test case file for demonstration
    dummy_data = [
        {
            "query": "How to install Docusaurus?",
            "expected_chunk_ids": [], # Will be filled manually or with a helper
            "expected_fragments": ["npx create-docusaurus"],
            "is_out_of_scope": False
        },
        {
            "query": "What is the capital of France?",
            "expected_chunk_ids": [],
            "expected_fragments": ["Paris"],
            "is_out_of_scope": True,
            "min_score_threshold": 0.5
        }
    ]
    dummy_file_path = "backend/data/retrieval_test_cases.json"
    Path(dummy_file_path).parent.mkdir(parents=True, exist_ok=True)
    with open(dummy_file_path, 'w', encoding='utf-8') as f:
        json.dump(dummy_data, f, indent=2)
    
    try:
        test_cases = load_retrieval_test_cases(dummy_file_path)
        print(f"Loaded {len(test_cases)} test cases:")
        for tc in test_cases:
            print(f"- Query: {tc.query}, Out of Scope: {tc.is_out_of_scope}")
    except FileNotFoundError as e:
        print(e)
    except Exception as e:
        print(f"An error occurred: {e}")
