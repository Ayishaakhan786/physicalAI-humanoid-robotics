# Quickstart Guide: Agent-Based Retrieval & Answering Backend

**Feature**: 005-rag-agent-backend
**Date**: 2025-12-18

## Overview

This guide provides instructions to quickly set up, run, and interact with the Agent-Based Retrieval & Answering Backend. It covers environment setup, dependency installation, configuration, and testing the `/query` API endpoint locally.

## 1. Prerequisites

Before you begin, ensure you have the following:

-   **Existing `backend/` directory**: Containing the Python project setup from Spec-1/Spec-2 (including `uv` environment, `pyproject.toml`).
-   **Populated Qdrant Collection**: The `rag_embedding` collection in Qdrant Cloud must be populated with data from a successful run of the Spec-1 ingestion pipeline.
-   **Environment Variables**:
    -   `OPENAI_API_KEY`: Your OpenAI API Key (or a compatible LLM API key for OpenAI Agents SDK).
    -   `COHERE_API_KEY`: Your Cohere API Key (must match the one used in Spec-1).
    -   `QDRANT_URL`: Your Qdrant Cloud cluster URL.
    -   `QDRANT_API_KEY`: Your Qdrant Cloud API Key.

## 2. Setup Project Environment

1.  **Navigate to the `backend/` directory**:
    ```bash
    cd D:\CODING\physicalAI-Humanoid-robotics\backend
    ```

2.  **Ensure virtual environment is active and dependencies are installed**:
    If not already active, activate your `uv` virtual environment:
    ```bash
    # On Windows
    .venv\Scripts\activate
    
    # On macOS/Linux
    source .venv/bin/activate
    ```
    Install new dependencies (FastAPI, Uvicorn, OpenAI Agents SDK, etc.):
    ```bash
    uv sync
    ```
    *(You will need to update `pyproject.toml` with new dependencies for FastAPI, Uvicorn, and OpenAI Agents SDK prior to running `uv sync`)*

## 3. Configuration

Ensure the `.env` file in your `backend/` directory contains the required environment variables:

-   `OPENAI_API_KEY`: Your OpenAI API Key.
-   `COHERE_API_KEY`: Your Cohere API Key.
-   `QDRANT_URL`: Your Qdrant Cloud cluster URL.
-   `QDRANT_API_KEY`: Your Qdrant Cloud API Key.

## 4. Run the FastAPI Backend

1.  **Ensure you are in the `backend/` directory and the virtual environment is active.**
2.  **Start the FastAPI application using Uvicorn**:
    ```bash
    uvicorn main:app --host 0.0.0.0 --port 8000 --reload
    ```
    *(The main FastAPI application instance will be named `app` in `main.py`)*

    The backend will start and be accessible at `http://localhost:8000`.

## 5. Test the `/query` API Endpoint

You can test the API endpoint using tools like `curl`, Postman, Insomnia, or by accessing the interactive API documentation at `http://localhost:8000/docs` or `http://localhost:8000/redoc`.

### Example `curl` request:

```bash
curl -X POST "http://localhost:8000/query" \
     -H "Content-Type: application/json" \
     -d 
           "question": "What is the primary purpose of the ROS2 framework?"
         
```

### Expected Response:

A structured JSON response, similar to:

```json
{
  "answer": "The primary purpose of the ROS2 framework is to enable the development of distributed robotic applications...",
  "sources": [
    {
      "source_url": "https://physical-ai-humanoid-robotics-umber.vercel.app/docs/module-01-ros2/ros-architecture",
      "document_section": "module-01-ros2",
      "heading": "ROS2 Architecture Overview",
      "chunk_id": "some-uuid-1"
    }
  ]
}
```

Or, for an unanswerable question:

```json
{
  "answer": "I am sorry, but I cannot answer your question based on the information available in the book. The retrieved content does not contain sufficient details to form a grounded response.",
  "sources": []
}
```

---
