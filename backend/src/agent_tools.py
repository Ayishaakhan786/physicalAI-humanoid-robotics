from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient, models
from src.retrieval_service import retrieve_chunks # Using Spec-2 retrieval pipeline
from src.embedding import embed_query
from src.utils.config import Config
from src.models import RetrievedResult, Source
import logging

# Configure basic logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def get_retrieval_tool(qdrant_client: QdrantClient, cohere_api_key: str, collection_name: str):
    """
    Returns a custom retrieval tool function for the OpenAI RAG agent.
    This tool queries Qdrant and returns top-k relevant chunks.
    """
    def retrieve_documents(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieves top_k relevant document chunks from Qdrant based on a natural language query.
        
        :param query: The natural language query to search for.
        :param top_k: The number of top relevant chunks to retrieve.
        :return: A list of dictionaries, each representing a retrieved chunk with its text and metadata.
        """
        logging.info(f"Agent Tool: retrieve_documents called with query='{query}', top_k={top_k}")
        try:
            query_vector = embed_query(query, cohere_api_key)
            
            retrieved_results: List[RetrievedResult] = retrieve_chunks(
                qdrant_client=qdrant_client,
                collection_name=collection_name,
                query_vector=query_vector,
                limit=top_k
            )
            
            # Convert RetrievedResult objects to a format suitable for the agent
            formatted_results = []
            for res in retrieved_results:
                source = Source(
                    source_url=res.retrieved_chunk_metadata.source_url,
                    document_section=res.retrieved_chunk_metadata.document_section,
                    heading=res.retrieved_chunk_metadata.heading,
                    chunk_id=res.chunk_id,
                    chunk_text_preview=res.retrieved_chunk_metadata.chunk_text_preview
                )
                formatted_results.append({
                    "text": res.retrieved_chunk_text,
                    "source": source.model_dump() # Convert pydantic model to dict
                })
            logging.info(f"Agent Tool: retrieve_documents returned {len(formatted_results)} chunks.")
            return formatted_results
        except Exception as e:
            logging.error(f"Agent Tool: Error in retrieve_documents tool: {e}")
            return [{"error": f"Failed to retrieve documents: {e}"}]

    return retrieve_documents

# Example usage of the tool (for testing purposes)
if __name__ == "__main__":
    from qdrant_client import QdrantClient
    
    try:
        Config.validate()
    except ValueError as e:
        logging.error(f"Configuration Error: {e}")
        exit(1)

    qdrant_client = QdrantClient(url=Config.QDRANT_URL, api_key=Config.QDRANT_API_KEY)
    cohere_api_key = Config.COHERE_API_KEY
    collection_name = "rag_embedding"

    retrieval_tool_func = get_retrieval_tool(qdrant_client, cohere_api_key, collection_name)

    sample_query = "What are the components of ROS2?"
    results = retrieval_tool_func(query=sample_query, top_k=2)
    
    if results and not results[0].get("error"):
        for i, res in enumerate(results):
            logging.info(f"\n--- Retrieved Chunk {i+1} ---")
            logging.info(f"Text: {res['text'][:200]}...")
            logging.info(f"Source URL: {res['source']['source_url']}")
    else:
        logging.error(f"Failed to retrieve documents for query '{sample_query}': {results[0].get('error') if results else 'No results'}")
