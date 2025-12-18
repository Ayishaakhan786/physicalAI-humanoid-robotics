# Data Model: Frontend and Backend Integration

**Feature**: 006-frontend-backend-integration
**Date**: 2025-12-18

## Overview

This document describes the key data entities involved in the Frontend and Backend Integration, outlining their structure, attributes, and relationships. It builds upon previous specifications and defines the data flow between the Docusaurus frontend and the FastAPI backend.

## Entities

### 1. FrontendQueryRequest

Represents the data payload sent from the frontend to the backend's `/query` endpoint. This extends the `QueryRequest` from Spec-3.

-   **`question`** (string): The user's natural language question.
-   **`selected_text_context`** (string, optional): Text selected by the user on the Docusaurus page.
-   **`current_page_url`** (HttpUrl): The URL of the Docusaurus page where the question originated.

### 2. FrontendChatState

Represents the state of the chatbot UI on the frontend.

-   **`is_chat_open`** (boolean): Indicates whether the chatbot UI is currently visible to the user.
-   **`messages`** (List[ChatMessage]): A historical log of user questions and agent responses.
-   **`current_input`** (string): The current text being typed by the user in the input field.
-   **`is_loading`** (boolean): A flag indicating if a response from the backend is currently awaited.
-   **`error_message`** (string, optional): Stores an error message to be displayed to the user if an issue occurs.

### 3. ChatMessage

Represents a single message within the `FrontendChatState`, encapsulating either a user's question or an agent's response.

-   **`type`** (enum: 'user', 'agent'): Indicates whether the message is from the user or the RAG agent.
-   **`content`** (string): The text content of the message (user question or agent answer/refusal).
-   **`sources`** (List[Source], optional): For agent messages, a list of `Source` objects providing citations (mirrors `AgentResponse.sources`).
-   **`timestamp`** (datetime): The time the message was sent or received.

### 4. BackendResponse (FastAPI AgentResponse from Spec-3)

The structured JSON response received from the FastAPI backend. This directly uses the `AgentResponse` model defined in Spec-3.

-   **`answer`** (string): The agent's generated answer or refusal message.
-   **`sources`** (List[Source], optional): List of source objects detailing the retrieved chunks.

### 5. FastAPI Backend API

The `/query` endpoint exposed by the backend (Spec-3) for RAG question answering.

-   **`url`** (HttpUrl): The configurable URL of the backend API (e.g., `http://localhost:8000/query`).
-   **`method`** (string): `POST`.
-   **`request_schema`**: `QueryRequest` (Spec-3 model).
-   **`response_schema`**: `AgentResponse` (Spec-3 model).
-   **`error_schema`**: `ErrorResponse` (Spec-3 model).

## Relationships

-   A `User Query` originating from the `Frontend Chatbot UI` is encapsulated within a `FrontendQueryRequest` and sent to the `FastAPI Backend API`.
-   The `FastAPI Backend API` returns a `Backend Response` (`AgentResponse`) to the `Frontend Chatbot UI`.
-   The `Frontend Chatbot UI` manages its `FrontendChatState`, which includes `ChatMessage` history.
-   `Selected Text Context` is an optional augmentation to the `User Query` in the `FrontendQueryRequest`.

## Validation Rules (Examples)

-   `FrontendQueryRequest.question`: Must not be empty.
-   `FrontendQueryRequest.selected_text_context`: Should not exceed a certain character limit to prevent excessively long context.
-   `ChatMessage.content`: Must not be empty.
-   Frontend should handle network errors and API errors (e.g., 400, 500 from `FastAPI Backend API`) gracefully, updating `FrontendChatState.error_message`.
-   The `Frontend Chatbot UI` must dynamically configure the `FastAPI Backend API` `url` using environment variables.