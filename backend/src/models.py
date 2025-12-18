from datetime import datetime
from typing import List, Optional, Dict, Any
from pydantic import BaseModel, Field, HttpUrl

class VectorEmbedding(BaseModel):
    vector: List[float]
    dimensionality: int
    model_name: str
    generated_at: datetime = Field(default_factory=datetime.utcnow)

class EmbeddingMetadata(BaseModel):
    source_url: HttpUrl
    document_section: Optional[str] = None
    heading: Optional[str] = None
    chunk_id: str
    chunk_text_preview: str = Field(..., max_length=500) # Max length for preview
    page_title: Optional[str] = None
    last_modified_date: Optional[datetime] = None
    updated_at: datetime = Field(default_factory=datetime.utcnow)

class RetrievedResult(BaseModel):
    chunk_id: str
    score: float
    retrieved_chunk_metadata: EmbeddingMetadata
    retrieved_chunk_text: str

class RetrievalTestCase(BaseModel):
    query: str
    expected_chunk_ids: List[str] = Field(default_factory=list)
    expected_fragments: Optional[List[str]] = None
    min_score_threshold: float = 0.0
    is_out_of_scope: bool = False

# New models for Agent Backend API
class QueryRequest(BaseModel):
    question: str

class Source(BaseModel):
    source_url: HttpUrl
    document_section: Optional[str] = None
    heading: Optional[str] = None
    chunk_id: str
    chunk_text_preview: str = Field(..., max_length=500) # Max length for preview

class AgentResponse(BaseModel):
    answer: str
    sources: Optional[List[Source]] = None

class ErrorResponse(BaseModel):
    detail: str
    status_code: int

# Example usage (for testing models)
# if __name__ == "__main__":
#     # Test VectorEmbedding
#     vec_embed = VectorEmbedding(
#         vector=[0.1, 0.2, 0.3],
#         dimensionality=3,
#         model_name="test-model"
#     )
#     print(f"VectorEmbedding: {vec_embed.model_dump_json(indent=2)}")

#     # Test EmbeddingMetadata
#     metadata = EmbeddingMetadata(
#         source_url="https://example.com/docs/page1",
#         chunk_id="abc-123",
#         chunk_text_preview="This is a preview of the chunk text."
#     )
#     print(f"EmbeddingMetadata: {metadata.model_dump_json(indent=2)}")
