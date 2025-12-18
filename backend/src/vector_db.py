from typing import Any, List, Tuple
from qdrant_client import QdrantClient, models
from src.models import VectorEmbedding, EmbeddingMetadata

def create_qdrant_collection(
    qdrant_client: QdrantClient,
    collection_name: str,
    vector_size: int,
    distance: models.Distance = models.Distance.COSINE
) -> None:
    """
    Creates a new Qdrant collection with a specified name and vector size if it doesn't already exist.
    """
    if not qdrant_client.collection_exists(collection_name=collection_name):
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=vector_size, distance=distance),
            # You might want to add payload_m_config for indexing specific fields from EmbeddingMetadata
            # For now, we'll keep it simple.
        )
        print(f"Collection '{collection_name}' created with vector size {vector_size} and distance {distance.value}.")
    else:
        print(f"Collection '{collection_name}' already exists.")

def save_chunks_to_qdrant(
    qdrant_client: QdrantClient,
    collection_name: str,
    embedded_chunks: List[Tuple[VectorEmbedding, EmbeddingMetadata]]
) -> None:
    """
    Stores a list of embedded chunks (vectors + metadata) into the specified Qdrant collection.
    """
    if not embedded_chunks:
        print("No embedded chunks to save.")
        return

    points = []
    for vec_embed, embed_meta in embedded_chunks:
        points.append(
            models.PointStruct(
                id=embed_meta.chunk_id, # Use chunk_id as Qdrant point ID
                vector=vec_embed.vector,
                payload=embed_meta.model_dump() # Pydantic model_dump directly as payload
            )
        )
    
    batch_size = 10 # Define a reasonable batch size
    for i in range(0, len(points), batch_size):
        batch = points[i : i + batch_size]
        qdrant_client.upsert(
            collection_name=collection_name,
            points=batch,
            wait=True
        )
        print(f"  -> Saved batch {i // batch_size + 1}/{(len(points) + batch_size - 1) // batch_size} ({len(batch)} points).")
    print(f"Successfully saved {len(points)} embedded chunks to collection '{collection_name}'.")

# Example Usage
if __name__ == "__main__":
    from src.utils.config import Config
    
    # Load and validate config
    try:
        Config.validate()
    except ValueError as e:
        print(f"Configuration Error: {e}")
        exit(1)

    # Initialize Qdrant Client
    qdrant_client = QdrantClient(
        url=Config.QDRANT_URL,
        api_key=Config.QDRANT_API_KEY,
    )
    
    test_collection_name = "test_rag_embedding"
    test_vector_size = 3 # Must match the size of test vectors

    try:
        # 1. Create collection
        create_qdrant_collection(qdrant_client, test_collection_name, test_vector_size)

        # 2. Prepare dummy embedded chunks
        from src.models import VectorEmbedding, EmbeddingMetadata
        from datetime import datetime
        from pydantic import HttpUrl

        dummy_embedded_chunks = [
            (
                VectorEmbedding(vector=[0.1, 0.2, 0.3], dimensionality=test_vector_size, model_name="test-model"),
                EmbeddingMetadata(source_url=HttpUrl("https://example.com/test1"), chunk_id="test-chunk-1", chunk_text_preview="Preview 1")
            ),
            (
                VectorEmbedding(vector=[0.4, 0.5, 0.6], dimensionality=test_vector_size, model_name="test-model"),
                EmbeddingMetadata(source_url=HttpUrl("https://example.com/test2"), chunk_id="test-chunk-2", chunk_text_preview="Preview 2")
            )
        ]

        # 3. Save chunks
        save_chunks_to_qdrant(qdrant_client, test_collection_name, dummy_embedded_chunks)

        # 4. Search example (optional, just to show client works)
        search_result = qdrant_client.search(
            collection_name=test_collection_name,
            query_vector=[0.15, 0.25, 0.35],
            limit=1
        )
        print(f"\nSearch result: {search_result}")

        # Clean up (delete collection after test)
        # qdrant_client.delete_collection(collection_name=test_collection_name)
        # print(f"Collection '{test_collection_name}' deleted.")

    except Exception as e:
        print(f"An error occurred during Qdrant example usage: {e}")
