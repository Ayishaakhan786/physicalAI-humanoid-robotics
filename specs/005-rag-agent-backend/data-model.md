# Data Model: Agent-Based Retrieval & Answering Backend

**Feature**: 005-rag-agent-backend
**Date**: 2025-12-18

## Overview

This document describes the key data entities involved in the Agent-Based Retrieval & Answering Backend, outlining their structure, attributes, and relationships. These entities utilize or extend models from previous specifications and define the API interaction with the FastAPI endpoint.

## Entities

### 1. QueryRequest

Represents the incoming request body for the FastAPI `/query` endpoint.

-   **`question`** (string): The user's natural language question.

### 2. Source

Represents the metadata of a retrieved chunk, used for citations in the `AgentResponse`. This mirrors key fields from `EmbeddingMetadata` in Spec-1.

-   **`source_url`** (HttpUrl): URL of the original Docusaurus page.
-   **`document_section`** (string, optional): Section from where the chunk originated.
-   **`heading`** (string, optional): Closest heading of the chunk.
-   **`chunk_id`** (string): Unique identifier for the chunk.
-   **`chunk_text_preview`** (string): A short preview of the chunk's text (optional, for debugging/verbose responses).

### 3. AgentResponse

Represents the structured JSON response returned by the FastAPI `/query` endpoint.

-   **`answer`** (string): The agent's generated answer to the question, or a refusal message if context is insufficient.
-   **`sources`** (List[Source], optional): A list of `Source` objects detailing the retrieved chunks used to formulate the answer. This list is empty if the agent refuses to answer or no relevant sources were found.

### 4. ErrorResponse

Represents a structured error message returned by the FastAPI endpoint in case of API errors or agent failures.

-   **`detail`** (string): A descriptive error message.
-   **`status_code`** (int): The HTTP status code associated with the error (e.g., 400, 500).

### 5. RAGAgentState

Represents the internal state and configuration of the RAG agent orchestrated by OpenAI Agents SDK.

-   **`llm_model_name`** (string): The name of the underlying LLM model used by the agent (e.g., `gpt-4o`, `gpt-3.5-turbo`).
-   **`temperature`** (float): The sampling temperature used by the LLM.
-   **`system_prompt`** (string): The initial system message guiding the agent's behavior, emphasizing grounding and refusal.
-   **`tools`** (List[string]): A list of names of tools available to the agent (e.g., `retrieve_documents`).
-   **`max_iterations`** (int): Maximum steps the agent can take.

### Relationships

-   A `FastAPI Request` (containing `QueryRequest`) triggers the `RAG Agent`.
-   The `RAG Agent` invokes the Spec-2 retrieval pipeline (which returns `Retrieved Context` / `RetrievedResult` objects).
-   The `RAG Agent` uses `Retrieved Context` to generate an `Agent Response`.
-   A `FastAPI Response` (containing `AgentResponse` or `ErrorResponse`) is returned to the user.

## Dependencies

-   **Spec-1 Entities**:
    -   `VectorEmbedding`, `EmbeddingMetadata` (used internally by retrieval pipeline).
-   **Spec-2 Entities**:
    -   `Query`, `Query Embedding` (used internally by retrieval pipeline).
    -   `RetrievedResult`, `RetrievalTestCase` (used by retrieval pipeline, `RetrievedResult` converted to `Source`).
    -   `Qdrant Search Request`, `Qdrant Search Response` (used internally by retrieval pipeline).

## Validation Rules (Examples)

-   `QueryRequest.question`: Must not be empty.
-   `AgentResponse.answer`: Must be non-empty if `sources` are provided. Must contain a refusal message if `sources` is empty and context was insufficient.
-   `AgentResponse.sources`: If present, `source_url` must be a valid HttpUrl and `chunk_id` must be a non-empty string.
-   FastAPI endpoint input validation using Pydantic will enforce schema adherence.