# ADR-0010: RAG Agent Orchestration and LLM Integration

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

-   **Status:** Proposed
-   **Date:** 2025-12-18
-   **Feature:** 005-rag-agent-backend
-   **Context:** The core of the RAG chatbot's question-answering capability relies on intelligently orchestrating retrieval, performing sophisticated reasoning over the retrieved context, and generating accurate, grounded responses. This involves selecting an appropriate agent framework to manage this workflow and integrating with a powerful Large Language Model (LLM). The choice of agent orchestration framework impacts development complexity, flexibility, and critically, the ability to enforce RAG constraints like strict grounding and refusal to answer with insufficient context.

## Decision

The RAG agent's orchestration and reasoning will be powered by the OpenAI Agents SDK. A compatible LLM (e.g., from OpenAI API, or other providers via `liteLLM` for broader compatibility) will be configured within the Agents SDK to perform the core reasoning and answer generation tasks. The agent will be designed to use custom tools, specifically one that interfaces with our Qdrant retrieval pipeline (Spec-2 validated), to dynamically gather context before formulating an answer.

## Consequences

### Positive

*   **Adherence to Constraint**: Directly fulfills the explicit requirement to use OpenAI Agents SDK for agent orchestration, aligning with project specifications.
*   **Robust Framework**: OpenAI Agents SDK provides a structured, high-level, and often performant approach to building agentic workflows, simplifying the development of complex RAG logic and decision-making processes.
*   **Tool Integration**: The SDK is specifically designed for easy and structured integration of custom tools, which is ideal for incorporating our Spec-2 validated retrieval pipeline as an agent capability.
*   **LLM Flexibility**: While often associated with OpenAI's own models, the SDK is designed to be flexible, allowing for potential integration with other compatible LLMs via standardized interfaces, offering future choices.
*   **Focus on Reasoning**: The framework abstracts away much of the boilerplate, allowing developers to focus on defining agent behavior, tools, and prompt engineering strategies rather than low-level LLM interaction.

### Negative

*   **Vendor Lock-in**: Reliance on OpenAI Agents SDK creates a dependency on a specific framework. While powerful, this could limit future choices or require significant refactoring if a different agent paradigm or vendor becomes more suitable.
*   **Abstraction Layer**: The SDK introduces an abstraction layer over raw LLM calls. While beneficial for development speed, it might obscure fine-grained control over LLM interactions or agent behavior in highly customized or complex scenarios.
*   **Learning Curve**: Developers new to the project or to agentic programming might face a learning curve to understand the OpenAI Agents SDK's concepts, patterns, and debugging tools.
*   **LLM Dependency**: The performance, cost, and availability of the RAG agent are inherently tied to the chosen LLM, which is an external dependency with its own API limits and pricing structures.

## Alternatives Considered

*   **Alternative 1: LangChain Agents**:
    *   **Description**: Utilize LangChain's comprehensive set of agentic functionalities, including tools, agents, and chains, for orchestrating RAG workflows.
    *   **Pros**: Highly mature and popular, broad community support, extensive integrations with various LLMs and tools, offers significant flexibility and customizability in agent design.
    *   **Cons**: Can have a steeper learning curve due to its extensive API and modularity, might involve more boilerplate code for simple tasks, potentially less opinionated than OpenAI Agents SDK for certain agent patterns.
*   **Alternative 2: LlamaIndex Agents**:
    *   **Description**: Use LlamaIndex for building and orchestrating RAG agents, which is particularly strong in data indexing and retrieval aspects, offering agents built around data sources.
    *   **Pros**: Excellent for integrating with various data sources and document stores, robust RAG capabilities, strong focus on data-aware agents and querying.
    *   **Cons**: Might be considered overkill for pure agent orchestration if data indexing/retrieval is already handled by separate, well-defined modules (like our Spec-1/2 pipelines), may have a different agent interaction paradigm.
*   **Alternative 3: Custom Agent Logic**:
    *   **Description**: Implement the agent's reasoning loop, tool invocation, and response generation logic manually using direct LLM API calls and conditional programming without relying on a dedicated agent framework.
    *   **Pros**: Maximum control and flexibility over every aspect of the agent's behavior, no external framework dependency.
    *   **Cons**: High development effort, significantly more complex to implement robust error handling, prompt management, and state tracking; prone to errors and difficult to maintain and scale without a structured framework. Requires significant prompt engineering expertise.

## References

-   Feature Spec: D:\CODING\physicalAI-Humanoid-robotics\specs\005-rag-agent-backend\spec.md
-   Implementation Plan: D:\CODING\physicalAI-Humanoid-robotics\specs\005-rag-agent-backend\plan.md
-   Research: D:\CODING\physicalAI-Humanoid-robotics\specs\005-rag-agent-backend\research.md (R-001, R-002)
-   Related ADRs: None
-   Evaluator Evidence: None
