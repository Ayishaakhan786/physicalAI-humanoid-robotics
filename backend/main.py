import os
import logging
from typing import Optional, Dict, Any, List
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, status
from pydantic import ValidationError
from qdrant_client import QdrantClient

from src.utils.config import Config
from src.models import QueryRequest, AgentResponse, ErrorResponse, Source # Import new models including Source
from src.rag_agent import RAGAgent # Import the RAG agent

# Configure basic logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Global variables for Qdrant client and RAG agent
qdrant_client: Optional[QdrantClient] = None
rag_agent: Optional[RAGAgent] = None
collection_name: str = "rag_embedding" # Defined in plan.md

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup logic
    logging.info("FastAPI app starting up...")
    try:
        Config.validate()
    except ValueError as e:
        logging.error(f"Configuration Error: {e}")
        # Ideally, raise exception or exit, but FastAPI lifespan requires yielding
        raise RuntimeError(f"Configuration Error: {e}")

    global qdrant_client, rag_agent
    
    qdrant_client = QdrantClient(url=Config.QDRANT_URL, api_key=Config.QDRANT_API_KEY)
    logging.info("Qdrant Client initialized.")

    rag_agent = RAGAgent(
        openai_api_key=Config.OPENAI_API_KEY,
        qdrant_client=qdrant_client,
        cohere_api_key=Config.COHERE_API_KEY,
        collection_name=collection_name
    )
    logging.info("RAG Agent initialized.")
    yield
    # Shutdown logic
    logging.info("FastAPI app shutting down.")

app = FastAPI(
    title="RAG Chatbot Backend",
    description="Agent-Based Retrieval & Answering Backend for Physical AI & Humanoid Robotics book.",
    version="0.1.0",
    lifespan=lifespan
)

@app.post("/query", response_model=AgentResponse, responses={
    status.HTTP_400_BAD_REQUEST: {"model": ErrorResponse},
    status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorResponse},
})
async def query_endpoint(request: QueryRequest):
    """
    Accepts a user question and returns a grounded answer from the RAG agent.
    """
    if not rag_agent:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="RAG Agent not initialized."
        )
    
    try:
        # Generate response using the RAG agent
        agent_response_dict = rag_agent.generate_response(request.question)
        
        # Validate the agent's response against our Pydantic model
        # The agent_response_dict should already be compatible for AgentResponse model
        response_model = AgentResponse(
            answer=agent_response_dict.get("answer", "No answer provided."),
            sources=agent_response_dict.get("sources", [])
        )
        return response_model
    except ValidationError as ve:
        logging.error(f"Agent response validation error: {ve}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Invalid response format from agent: {ve}"
        )
    except Exception as e:
        logging.error(f"Error processing query '{request.question}': {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An internal error occurred: {e}"
        )