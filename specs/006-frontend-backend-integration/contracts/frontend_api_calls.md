# API Contracts: Frontend Interaction with RAG Backend

**Feature**: 006-frontend-backend-integration
**Date**: 2025-12-18
**Context**: This document defines the frontend's interaction with the existing RAG Agent Backend's `/query` endpoint. It describes the data structures used by the frontend to send questions and receive answers, building upon the `QueryRequest` and `AgentResponse` models defined in Spec-3.

## Frontend-Backend Communication

The frontend will communicate with the backend's `/query` endpoint using REST (JSON over HTTP).

### 1. Backend Endpoint Reference

The backend API endpoint being consumed is:

-   **Endpoint**: `/query`
-   **Method**: `POST`
-   **Description**: Accepts a user question and returns a grounded answer from the RAG agent.
-   **Request Body Schema**: Defined by `QueryRequest` (from `specs/005-rag-agent-backend/data-model.md`).
    -   **Extended Schema for Frontend**: The frontend will augment `QueryRequest` with `selected_text_context` and `current_page_url`. This will be internally mapped to the backend's `QueryRequest` which currently only expects `question`. **[NEEDS CLARIFICATION: The backend QueryRequest needs to be extended to accept `selected_text_context` and `current_page_url` if the agent is to use them.]**
-   **Response Body Schema**: Defined by `AgentResponse` (from `specs/005-rag-agent-backend/data-model.md`).
-   **Error Response Schema**: Defined by `ErrorResponse` (from `specs/005-rag-agent-backend/data-model.md`).

### 2. Frontend-Specific Data Structures

#### `FrontendQueryRequest`

This is the object the frontend will construct and send to the backend. It extends the backend's `QueryRequest` with additional context from the frontend.

-   **Schema**:
    ```json
    {
      "type": "object",
      "properties": {
        "question": {
          "type": "string",
          "description": "The user's natural language question about the book content.",
          "minLength": 1
        },
        "selected_text_context": {
          "type": "string",
          "description": "Optional text selected by the user on the Docusaurus page, providing additional context for the question.",
          "nullable": true
        },
        "current_page_url": {
          "type": "string",
          "format": "uri",
          "description": "The full URL of the Docusaurus page where the question originated.",
          "minLength": 1
        }
      },
      "required": ["question", "current_page_url"]
    }
    ```
-   **Example**:
    ```json
    {
      "question": "What is covered in the introduction?",
      "selected_text_context": "Docusaurus is a static site generator...",
      "current_page_url": "https://physical-ai-humanoid-robotics-umber.vercel.app/docs/intro"
    }
    ```

### 3. Frontend API Call Function

A client-side function to handle the HTTP POST request to the backend.

-   **`postQuery(question: string, selectedText?: string, pageUrl?: string) -> Promise<AgentResponse>`**:
    -   **Description**: Asynchronously sends a `FrontendQueryRequest` to the configured backend API endpoint.
    -   **Input**:
        -   `question` (string): The user's question.
        -   `selectedText` (string, optional): Selected text from the page.
        -   `pageUrl` (string, optional): Current Docusaurus page URL.
    -   **Output**:
        -   `Promise<AgentResponse>`: A promise that resolves with the `AgentResponse` on success, or rejects with an `ErrorResponse` (or equivalent structure) on failure.
    -   **Behavior**:
        -   Constructs `FrontendQueryRequest` payload.
        -   Fetches backend API URL from environment configuration.
        -   Handles network requests, JSON serialization/deserialization.
        -   Catches and maps HTTP errors to `ErrorResponse`.
        -   Configures CORS headers if necessary.

## Dependencies

-   **Backend `/query` endpoint**: Defined in `specs/005-rag-agent-backend/contracts/query_api.md`.
-   **Backend Models**: `QueryRequest`, `AgentResponse`, `ErrorResponse` (from `specs/005-rag-agent-backend/data-model.md` and `backend/src/models.py`).

## Notes

-   **[NEEDS CLARIFICATION]**: The backend `QueryRequest` model (defined in `specs/005-rag-agent-backend/data-model.md` and implemented in `backend/src/models.py`) needs to be extended to accept `selected_text_context` and `current_page_url`. Without these fields, the backend agent cannot process context-aware questions.
-   CORS on the FastAPI backend needs to be configured to allow requests from the Docusaurus frontend's origin (`localhost` for development, deployed URL for production).
-   The frontend will use native `fetch` API or a library like `Axios` for HTTP requests.
