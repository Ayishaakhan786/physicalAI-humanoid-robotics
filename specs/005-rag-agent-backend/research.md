# Research Findings: Agent-Based Retrieval & Answering Backend

**Feature**: 005-rag-agent-backend
**Date**: 2025-12-18

## Overview of Research Tasks

The following research tasks were identified to address technical unknowns and inform detailed design decisions for the Agent-Based Retrieval & Answering Backend.

## Findings

### R-001: OpenAI Agents SDK Setup and Usage

-   **Objective**: Understand how to set up, configure, and utilize the OpenAI Agents SDK for orchestrating a RAG workflow.
-   **Decision**: [NEEDS RESEARCH: Specific agent initialization, tool definition syntax, agent execution flow, and any necessary model/API key configurations.]
-   **Rationale**: [NEEDS RESEARCH: Justification for chosen agent configuration (e.g., model, temperature, max_iterations) to ensure grounding and refusal.]
-   **Alternatives Considered**: [NEEDS RESEARCH: LangChain Agents, LlamaIndex Agents, custom agent implementations.]

### R-002: Prompt Engineering for Grounding and Refusal

-   **Objective**: Develop effective system prompts and tool descriptions to guide the RAG agent to strictly ground answers in retrieved context and refuse to answer when context is insufficient.
-   **Decision**: [NEEDS RESEARCH: Specific system prompt templates, instructions for tool usage, and refusal mechanisms (e.g., specific phrases or output formats).]
-   **Rationale**: [NEEDS RESEARCH: How the chosen prompt strategy minimizes hallucination and maximizes grounding.]
-   **Alternatives Considered**: [NEEDS RESEARCH: Different prompt styles, few-shot examples for grounding/refusal, alternative model instructions.]

### R-003: Integrating Custom Retrieval Tool with OpenAI Agents SDK

-   **Objective**: Determine the best method to integrate our Spec-2 validated retrieval pipeline (`retrieve_chunks` and `embed_query`) as a custom tool within the OpenAI Agents SDK.
-   **Decision**: [NEEDS RESEARCH: How to define the retrieval tool's function signature, input schema, and output format for the agent. How to pass Qdrant/Cohere clients/keys to the tool.]
-   **Rationale**: [NEEDS RESEARCH: Justification for the chosen integration method to ensure seamless agent access to the retrieval capabilities.]
-   **Alternatives Considered**: [NEEDS RESEARCH: Using a pre-built retrieval tool from OpenAI Agents SDK (if compatible), alternative tool definitions.]

### R-004: FastAPI Best Practices for RAG Backend

-   **Objective**: Build a robust FastAPI application with a single `/query` endpoint, structured request/response models, dependency injection, and comprehensive error handling.
-   **Decision**: [NEEDS RESEARCH: Specific Pydantic models for `QueryRequest` and `AgentResponse`, FastAPI routing, dependency injection patterns for Qdrant/Cohere clients and the RAG agent, and error handling for API-specific exceptions.]
-   **Rationale**: [NEEDS RESEARCH: Justification for the chosen FastAPI architecture to ensure scalability, maintainability, and production readiness.]
-   **Alternatives Considered**: [NEEDS RESEARCH: Flask, Starlette, other Python web frameworks.]

---
