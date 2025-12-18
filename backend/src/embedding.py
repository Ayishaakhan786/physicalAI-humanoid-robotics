from typing import List, Dict, Any, Tuple
from cohere import Client

from datetime import datetime
from src.models import VectorEmbedding, EmbeddingMetadata # Assuming these models are defined

def embed_chunks(chunks: List[Dict[str, Any]], cohere_api_key: str) -> List[Tuple[VectorEmbedding, EmbeddingMetadata]]:
    """
    Generates vector embeddings for a list of text chunks using the Cohere API.
    
    Args:
        chunks: A list of dictionaries, where each dictionary represents a chunk
                and includes at least 'text', 'source_url', 'chunk_id', etc.
        cohere_api_key: The API key for accessing the Cohere embedding service.
        
    Returns:
        A list of tuples, where each tuple contains a VectorEmbedding instance
        and an EmbeddingMetadata instance.
    """
    if not chunks:
        return []

    co = Client(cohere_api_key)
    texts_to_embed = [chunk["text"] for chunk in chunks]
    
    # Cohere has a limit on the number of texts per request (e.g., 96 for embed-english-v3.0)
    # and a max token limit. For simplicity, we'll send all at once.
    # A robust implementation would batch these requests.
    try:
        response = co.embed(
            texts=texts_to_embed,
            model="embed-english-v3.0", # Using a common stable model
            input_type="classification" # or "search_document", "search_query"
        )
        
        embedded_results: List[Tuple[VectorEmbedding, EmbeddingMetadata]] = []
        for i, embedding_vector in enumerate(response.embeddings):
            chunk = chunks[i]
            
            # Create VectorEmbedding instance
            vec_embed = VectorEmbedding(
                vector=embedding_vector,
                dimensionality=len(embedding_vector),
                model_name="embed-english-v3.0",
                generated_at=datetime.utcnow()
            )

            # Create EmbeddingMetadata instance
            # Ensure chunk_text_preview is truncated if necessary
            chunk_text_preview = chunk["text"]
            if len(chunk_text_preview) > 500:
                chunk_text_preview = chunk_text_preview[:497] + "..."

            embed_metadata = EmbeddingMetadata(
                source_url=chunk["source_url"],
                document_section=chunk.get("document_section"),
                heading=chunk.get("heading"),
                chunk_id=chunk["chunk_id"],
                chunk_text_preview=chunk_text_preview,
                page_title=chunk.get("page_title"),
                updated_at=datetime.utcnow()
            )
            embedded_results.append((vec_embed, embed_metadata))
            
        return embedded_results
        
    except Exception as e:
        print(f"Error embedding chunks with Cohere: {e}")
        raise

def embed_query(query: str, cohere_api_key: str) -> List[float]:
    """
    Generates a vector embedding for a single query string using the Cohere API.
    """
    co = Client(cohere_api_key)
    response = co.embed(
        texts=[query],
        model="embed-english-v3.0", # Must match model used for chunk embeddings
        input_type="search_query"
    )
    return response.embeddings[0]

# Example usage
if __name__ == "__main__":
    # This requires a valid COHERE_API_KEY to be set in environment variables
    # or passed directly.
    from src.utils.config import Config
    Config.validate()
    cohere_api_key = Config.COHERE_API_KEY
    
    sample_chunks = [
        {"text": "This is the first sentence about Docusaurus.", "source_url": "https://example.com/doc1", "chunk_id": "1", "page_title": "Doc1"},
        {"text": "The quick brown fox jumps over the lazy dog.", "source_url": "https://example.com/doc2", "chunk_id": "2", "page_title": "Doc2", "document_section": "sectionA"},
    ]

    try:
        embedded_chunks_with_metadata = embed_chunks(sample_chunks, cohere_api_key)
        for vec, meta in embedded_chunks_with_metadata:
            print(f"Chunk ID: {meta.chunk_id}, Dimensionality: {vec.dimensionality}, Model: {vec.model_name}")
            print(f"Preview: {meta.chunk_text_preview}")
            # print(f"Vector (first 5): {vec.vector[:5]}...")
            print("-" * 20)
        
        # Test embed_query
        query_vec = embed_query("What is Docusaurus?", cohere_api_key)
        print(f"\nQuery embedding for 'What is Docusaurus?' (first 5): {query_vec[:5]}...")

    except Exception as e:
        print(f"An error occurred during example usage: {e}")
