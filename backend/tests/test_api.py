import pytest
from httpx import AsyncClient
from main import app # Assuming app is exposed from main.py
from src.models import QueryRequest, AgentResponse, ErrorResponse # Import response models

# Fixture for asynchronous client for FastAPI testing
@pytest.fixture(scope="module")
async def async_client():
    async with AsyncClient(app=app, base_url="http://test") as client:
        yield client

@pytest.mark.asyncio
async def test_query_valid_question(async_client: AsyncClient):
    """
    Test the /query endpoint with a valid question.
    This test assumes the RAG agent might provide a refusal if Qdrant is not populated
    or OpenAI API key is missing. So we're testing the structure of the response.
    """
    test_question = "What are the core components of ROS2?"
    response = await async_client.post("/query", json={"question": test_question})

    assert response.status_code == 200
    try:
        agent_response = AgentResponse(**response.json())
        assert isinstance(agent_response.answer, str)
        assert agent_response.answer != ""
        # Sources might be empty if refusal, so check if present and correct type
        if agent_response.sources is not None:
            assert isinstance(agent_response.sources, list)
            for source in agent_response.sources:
                assert source.source_url is not None # HttpUrl validation happens on model creation
                assert isinstance(source.chunk_id, str)
    except Exception as e:
        pytest.fail(f"Response did not match AgentResponse model: {e} - Response: {response.json()}")

@pytest.mark.asyncio
async def test_query_empty_question(async_client: AsyncClient):
    """
    Test the /query endpoint with an empty question, expecting a 400 Bad Request.
    """
    response = await async_client.post("/query", json={"question": ""})

    assert response.status_code == 422 # FastAPI's default for validation errors
    error_response = response.json()
    assert "detail" in error_response
    assert any("field required" in err["msg"].lower() for err in error_response["detail"]) # Check for Pydantic validation message

@pytest.mark.asyncio
async def test_query_missing_question_field(async_client: AsyncClient):
    """
    Test the /query endpoint with missing question field, expecting a 400 Bad Request.
    """
    response = await async_client.post("/query", json={})

    assert response.status_code == 422 # FastAPI's default for validation errors
    error_response = response.json()
    assert "detail" in error_response
    assert any("field required" in err["msg"].lower() for err in error_response["detail"])

@pytest.mark.asyncio
async def test_query_malformed_json(async_client: AsyncClient):
    """
    Test the /query endpoint with malformed JSON, expecting a 400 Bad Request.
    """
    response = await async_client.post("/query", content="this is not json")

    assert response.status_code == 400
    error_response = ErrorResponse(**response.json())
    assert "detail" in error_response.model_dump()
    assert "JSON decode error" in error_response.detail

# Additional tests can be added here for:
# - Out-of-scope questions (expecting refusal)
# - Agent internal errors (expecting 500)
# - Performance (latency - requires more advanced testing setup)
