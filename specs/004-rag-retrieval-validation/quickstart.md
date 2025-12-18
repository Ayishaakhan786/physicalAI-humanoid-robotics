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
