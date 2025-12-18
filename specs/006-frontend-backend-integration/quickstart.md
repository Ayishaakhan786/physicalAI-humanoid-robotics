# Quickstart Guide: Frontend and Backend Integration

**Feature**: 006-frontend-backend-integration
**Date**: 2025-12-18

## Overview

This guide provides instructions to quickly set up, run, and integrate the RAG Chatbot UI into the Docusaurus frontend and ensure its communication with the FastAPI backend. It covers environment setup, component integration, configuration, and testing.

## 1. Prerequisites

Before you begin, ensure you have the following:

-   **Running FastAPI Backend**: The Spec-3 RAG Agent Backend must be running and accessible (e.g., `http://localhost:8000`).
-   **Configured Backend URL**: The `RAG_BACKEND_URL` environment variable (or equivalent) must point to your running backend.
-   **Docusaurus Project**: An existing Docusaurus v2/v3 project (your `book/` directory in this repository).

## 2. Setup Docusaurus Project

1.  **Navigate to the Docusaurus project root**:
    ```bash
    cd book
    ```

2.  **Install dependencies**:
    ```bash
    npm install
    ```
    or
    ```bash
    yarn install
    ```

## 3. Integrate Chatbot React Component

*(The exact integration method will be detailed in implementation, but generally involves:)*

1.  **Create Custom React Component**: Develop the `ChatbotUI` React component in `book/src/components/ChatbotUI/index.js` (or `tsx`). This component will handle:
    -   Displaying chat history.
    -   User input field.
    -   Sending questions to the backend.
    -   Rendering agent responses and sources.
    -   Capturing selected text on the page.

2.  **Embed Component in Docusaurus**:
    -   Identify a suitable place to embed the `ChatbotUI` (e.g., a global layout component, a custom theme component, or directly within MDX pages).
    -   For a global floating chat UI, a custom theme swizzling might be necessary (e.g., `docusaurus swizzle @docusaurus/theme-classic Layout --danger`).

## 4. Configure Backend URL

1.  **Local Development**: Create a `.env` file in your Docusaurus `book/` directory (or use `docusaurus.config.js` for environment variables).
    ```dotenv
    RAG_BACKEND_URL=http://localhost:8000/query
    ```
    *(Note: Docusaurus environment variable handling might require `VITE_` or `DANGEROUSLY_SET_` prefixes depending on configuration.)*

2.  **Deployment**: Configure the `RAG_BACKEND_URL` environment variable in your deployment pipeline (e.g., Vercel, Netlify) to point to your deployed FastAPI backend URL.

## 5. Implement Frontend API Calls

Within your `ChatbotUI` component or a dedicated service file:

-   Implement a JavaScript/TypeScript function (e.g., `postQuery`) to:
    -   Construct the `FrontendQueryRequest` payload (question, `selected_text_context`, `current_page_url`).
    -   Send a `POST` request to `${process.env.RAG_BACKEND_URL}`.
    -   Handle JSON serialization/deserialization.
    -   Process `AgentResponse` and `ErrorResponse`.
    -   Example usage:
        ```javascript
        async function postQuery(question, selectedText, currentPageUrl) {
          const response = await fetch(process.env.RAG_BACKEND_URL, {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              question: question,
              selected_text_context: selectedText,
              current_page_url: currentPageUrl
            }),
          });
          if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.detail || 'API error');
          }
          return response.json();
        }
        ```

## 6. Run Docusaurus Locally

1.  **Navigate to the Docusaurus project root**:
    ```bash
    cd book
    ```
2.  **Start the development server**:
    ```bash
    npm start
    ```
    or
    ```bash
    yarn start
    ```
    Access the Docusaurus site, and the chatbot UI, at `http://localhost:3000`.

## 7. Test Functionality

-   **General Questions**: Test asking questions about the book content.
-   **Context-Aware Questions**: Select text on a page, then ask a question.
-   **Out-of-Scope Questions**: Verify the chatbot refuses to answer.
-   **API Error Handling**: Test what happens if the backend is down or returns an error.

---
