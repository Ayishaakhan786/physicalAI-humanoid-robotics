# API Contracts: RAG Agent Backend - `/query` Endpoint

**Feature**: 005-rag-agent-backend
**Date**: 2025-12-18
**Context**: This document defines the external-facing API contract for the RAG Agent Backend's `/query` endpoint. This endpoint serves as the primary interface for users or frontend applications to interact with the RAG agent for question answering.

## Endpoint: `/query`

### Method: `POST`

### Description

Accepts a user question as a JSON payload, processes it through the RAG agent (which includes retrieval from Qdrant and LLM-based reasoning), and returns a structured JSON response containing the agent's grounded answer or a refusal message, along with source citations.

### Request

-   **URL**: `/query`
-   **Headers**: `Content-Type: application/json`
-   **Body**: `QueryRequest` Pydantic model
    -   **Schema**:
        ```json
        {
          "type": "object",
          "properties": {
            "question": {
              "type": "string",
              "description": "The user's natural language question about the book content.",
              "minLength": 1
            }
          },
          "required": ["question"]
        }
        ```
    -   **Example**:
        ```json
        {
          "question": "What are the main components of a ROS2 system?"
        }
        ```

### Responses

#### Success: `200 OK`

-   **Body**: `AgentResponse` Pydantic model
    -   **Schema**:
        ```json
        {
          "type": "object",
          "properties": {
            "answer": {
              "type": "string",
              "description": "The agent's generated answer, or a message indicating inability to answer due to insufficient context."
            },
            "sources": {
              "type": "array",
              "items": {
                "type": "object",
                "properties": {
                  "source_url": {
                    "type": "string",
                    "format": "uri",
                    "description": "The URL of the original Docusaurus page where the chunk was found."
                  },
                  "document_section": {
                    "type": "string",
                    "description": "The section of the document where the chunk originated (e.g., 'module-01-ros2').",
                    "nullable": true
                  },
                  "heading": {
                    "type": "string",
                    "description": "The closest main heading to the chunk.",
                    "nullable": true
                  },
                  "chunk_id": {
                    "type": "string",
                    "description": "The unique identifier of the retrieved chunk."
                  },
                  "chunk_text_preview": {
                    "type": "string",
                    "description": "A short preview of the chunk's text.",
                    "nullable": true
                  }
                },
                "required": ["source_url", "chunk_id"]
              },
              "description": "A list of source chunks used to generate the answer. Empty if no answer or no sources were used."
            }
          },
          "required": ["answer", "sources"]
        }
        ```
    -   **Example (Grounded Answer)**:
        ```json
        {
          "answer": "The main components of a ROS2 system include nodes, topics, services, and actions. Nodes are processes that perform computation, topics are named buses for passing messages, services are for request/reply communication, and actions are for long-running tasks.",
          "sources": [
            {
              "source_url": "https://physical-ai-humanoid-robotics-umber.vercel.app/docs/module-01-ros2/nodes-topics",
              "document_section": "module-01-ros2",
              "heading": "ROS2 Nodes and Topics",
              "chunk_id": "abc-123",
              "chunk_text_preview": "Nodes are processes, topics are named buses..."
            },
            {
              "source_url": "https://physical-ai-humanoid-robotics-umber.vercel.app/docs/module-01-ros2/services-actions",
              "document_section": "module-01-ros2",
              "heading": "ROS2 Services and Actions",
              "chunk_id": "def-456",
              "chunk_text_preview": "Services are for request/reply, actions for long-running tasks..."
            }
          ]
        }
        ```
    -   **Example (Refusal)**:
        ```json
        {
          "answer": "I am sorry, but I cannot answer your question based on the information available in the book. The retrieved content does not contain sufficient details to form a grounded response.",
          "sources": []
        }
        ```

#### Error: `400 Bad Request`

-   **Body**: `ErrorResponse` Pydantic model
    -   **Schema**:
        ```json
        {
          "type": "object",
          "properties": {
            "detail": {
              "type": "string",
              "description": "A descriptive error message for the client."
            },
            "status_code": {
              "type": "integer",
              "description": "The HTTP status code (e.g., 400)."
            }
          },
          "required": ["detail", "status_code"]
        }
        ```
    -   **Example**:
        ```json
        {
          "detail": "Question field is required.",
          "status_code": 400
        }
        ```

#### Error: `500 Internal Server Error`

-   **Body**: `ErrorResponse` Pydantic model
    -   **Schema**: (Same as 400 Bad Request)
    -   **Example**:
        ```json
        {
          "detail": "An unexpected error occurred during agent processing.",
          "status_code": 500
        }
        ```
