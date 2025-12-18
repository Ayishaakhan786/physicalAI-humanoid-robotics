# Quickstart Guide: RAG Content Ingestion Pipeline

**Feature**: 003-rag-content-ingestion
**Date**: 2025-12-18

## Overview

This guide provides instructions to quickly set up and run the RAG Content Ingestion pipeline. It covers environment setup, dependency installation, configuration, and execution of the main script.

## 1. Prerequisites

Before you begin, ensure you have the following:

-   **Python 3.9+**: Installed on your system.
-   **`uv`**: The `uv` Python package manager (installable via `pip install uv` or similar).
-   **Cohere API Key**: An API key for accessing Cohere embedding models.
-   **Qdrant Cloud API Key & URL**: An API key and the cluster URL for your Qdrant Cloud instance.
-   **Deployed Docusaurus Website URL**: The base URL of the Docusaurus book you wish to ingest (e.g., `https://physical-ai-humanoid-robotics-umber.vercel.app/`).

## 2. Setup Project Environment

1.  **Navigate to the project root**:
    ```bash
    cd D:\CODING\physicalAI-Humanoid-robotics
    ```

2.  **Create a dedicated directory for the backend (if not already present)**:
    ```bash
    mkdir backend
    cd backend
    ```

3.  **Initialize `uv` environment and install dependencies**:
    Create a `pyproject.toml` or `requirements.txt` file (you will create this during implementation) listing dependencies like `requests`, `BeautifulSoup4`, `cohere`, `qdrant-client`.

    Using `pyproject.toml`:
    ```bash
    # Assuming pyproject.toml is configured with dependencies
    uv venv
    uv sync
    ```
    Or using `requirements.txt`:
    ```bash
    # Assuming requirements.txt is present
    uv venv
    uv pip install -r requirements.txt
    ```

4.  **Activate the virtual environment**:
    ```bash
    # On Windows
    .venv\Scripts\activate
    
    # On macOS/Linux
    source .venv/bin/activate
    ```

## 3. Configuration

Set the following environment variables. It is recommended to use a `.env` file and a library like `python-dotenv` for local development.

-   `COHERE_API_KEY`: Your Cohere API Key.
-   `QDRANT_URL`: Your Qdrant Cloud cluster URL (e.g., `https://<YOUR_CLUSTER_ID>.qdrant.tech`).
-   `QDRANT_API_KEY`: Your Qdrant Cloud API Key.
-   `DOCUSAURUS_BASE_URL`: The base URL of the Docusaurus website to ingest (e.g., `https://physical-ai-humanoid-robotics-umber.vercel.app/`).

## 4. Run the Pipeline

Once the environment is set up and configured, you can run the main ingestion script:

1.  **Ensure you are in the `backend/` directory and the virtual environment is active.**
2.  **Execute the main script**:
    ```bash
    python main.py
    ```

The script will proceed to:
-   Collect URLs from the Docusaurus site.
-   Extract text content from each page.
-   Chunk the text semantically.
-   Generate embeddings using Cohere.
-   Create a Qdrant collection named `rag_embedding` (if it doesn't exist).
-   Store the embeddings and metadata in Qdrant.
-   Perform a basic validation search.

## 5. Validation

After the script completes, you can verify the ingestion by:
-   Checking your Qdrant Cloud dashboard to confirm the `rag_embedding` collection exists and contains points.
-   Running `validate_ingestion` function manually from an interactive Python session or by extending `main.py` with specific test queries.

---



# Quickstart Guide: RAG Retrieval Pipeline & Validation

**Feature**: 004-rag-retrieval-validation
**Date**: 2025-12-18

## Overview

This guide provides instructions to quickly set up and run the RAG Retrieval Pipeline & Validation suite. It covers environment setup, dependency installation (if any new), configuration, and execution of the validation script.

## 1. Prerequisites

Before you begin, ensure you have the following:

-   **Existing `backend/` directory**: Containing the Python project setup from Spec-1 (including `uv` environment, `pyproject.toml`).
-   **Populated Qdrant Collection**: The `rag_embedding` collection in Qdrant Cloud must be populated with data, ideally from a successful run of the Spec-1 ingestion pipeline.
-   **Environment Variables**:
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
    Ensure all Spec-1 dependencies are installed:
    ```bash
    uv sync
    ```
    *No new dependencies are expected for this phase unless evaluation frameworks are added.*

## 3. Configuration

Ensure the `.env` file in your `backend/` directory contains the required environment variables:

-   `COHERE_API_KEY`: Your Cohere API Key.
-   `QDRANT_URL`: Your Qdrant Cloud cluster URL.
-   `QDRANT_API_KEY`: Your Qdrant Cloud API Key.

## 4. Run the Retrieval Validation Suite

This will involve executing a new Python script (e.g., `run_retrieval_tests.py`) that orchestrates the query embedding, retrieval, and evaluation.

1.  **Ensure you are in the `backend/` directory and the virtual environment is active.**
2.  **Execute the retrieval test script**:
    ```bash
    python run_retrieval_tests.py
    ```
    *(The actual filename `run_retrieval_tests.py` will be defined during implementation)*

The script will proceed to:
-   Load test queries.
-   Embed queries using Cohere.
-   Perform similarity searches against the `rag_embedding` Qdrant collection.
-   Validate results against predefined criteria (relevance, metadata, performance).
-   Log validation outcomes.

## 5. Interpreting Results

The script's output will provide details on each test case, including whether it passed or failed, retrieved chunks, scores, and any identified discrepancies. Review these logs to assess the retrieval pipeline's effectiveness.

---




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

