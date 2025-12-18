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
