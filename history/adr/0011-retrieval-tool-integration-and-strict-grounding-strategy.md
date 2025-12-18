# ADR-0011: Retrieval Tool Integration and Strict Grounding Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

-   **Status:** Proposed
-   **Date:** 2025-12-18
-   **Feature:** 005-rag-agent-backend
-   **Context:** A critical aspect of the RAG chatbot is its ability to accurately retrieve relevant information from the book content and then strictly ground its answers in that retrieved context. This involves effectively integrating the Spec-2 validated retrieval pipeline as a callable tool for the OpenAI RAG agent and defining mechanisms to ensure the agent adheres to strict grounding rules, including explicitly refusing to answer when context is insufficient. This strategy directly impacts the factual accuracy and trustworthiness of the RAG system.

## Decision

The Spec-2 validated retrieval pipeline (`embed_query` and `retrieve_chunks` functions) will be exposed to the OpenAI RAG agent as a custom tool, named `retrieve_documents`. This tool will accept a natural language query as input and return top-k relevant chunks from Qdrant, including their text and associated metadata (`source_url`, `chunk_id`, etc.). The RAG agent's system prompt will be carefully engineered to instruct it to:
1.  **Always call `retrieve_documents` first** when a user asks a question about the book content.
2.  **Strictly use only the information provided by the tool** (i.e., the retrieved context) for answering.
3.  **Explicitly state an inability to answer** if the retrieved context is insufficient, irrelevant, or contradictory to the user's question, rather than attempting to generate an answer from its general knowledge.

## Consequences

### Positive

*   **Strict Grounding Enforcement**: By integrating retrieval as an explicit tool and carefully engineering the system prompt, the agent is directed to primarily rely on retrieved content, significantly minimizing hallucinations and enhancing factual accuracy.
*   **Trustworthy Responses**: The explicit refusal mechanism for insufficient context builds user trust by clearly communicating the system's limitations and prevents misleading or ungrounded responses.
*   **Modular Integration**: Reuses the validated and tested retrieval pipeline components from Spec-2, ensuring consistency, leveraging existing efforts, and reducing the development burden for the retrieval aspect within the agent.
*   **Agent Control**: Provides the agent with a powerful, domain-specific tool, allowing it to autonomously decide when and how to retrieve information as part of its reasoning process.

### Negative

*   **Prompt Engineering Dependency**: The effectiveness of strict grounding and refusal is highly dependent on the quality and robustness of the system prompt and tool instructions. Subtle changes in phrasing can significantly alter agent behavior, requiring careful tuning and testing.
*   **Retrieval Performance Bottleneck**: The agent's overall response time will be directly impacted by the latency of the `retrieve_documents` tool (i.e., the time taken for Cohere query embedding and Qdrant search).
*   **False Refusals/Hallucinations**: Despite careful prompting, agents can sometimes misinterpret context or prompt instructions, potentially leading to either:
    *   **False Refusals**: Refusing to answer a question that could have been answered with the available context.
    *   **Weak Grounding/Hallucinations**: (Less likely with strict prompting, but possible) attempting to answer with weak grounding if the retrieved context is only superficially relevant, rather than truly refusing.
*   **Tool Complexity**: Defining the tool's schema and ensuring the agent correctly understands and uses it can be complex and requires iterative refinement.

## Alternatives Considered

*   **Alternative 1: Direct Context Injection (without explicit tool)**:
    *   **Description**: Instead of making retrieval an agent tool, the main application logic would first embed the user query, retrieve top-k chunks from Qdrant, and then directly inject these chunks as context into the LLM's prompt before passing it to the agent/LLM for answer generation.
    *   **Pros**: Simpler integration with the LLM (bypasses tool definition), potentially faster if agent overhead for tool calling is avoided.
    *   **Cons**: Less agent autonomy in deciding *when* to retrieve (retrieval is always performed upfront), less flexible if the retrieval strategy needs to change dynamically based on intermediate agent reasoning, harder to enforce explicit tool use for traceability within agent logs.
*   **Alternative 2: Less Strict Grounding**:
    *   **Description**: Allow the agent more leeway to use its general knowledge (pre-trained knowledge) to supplement retrieved context, or infer answers even from partially relevant information. The system prompt would be less restrictive on grounding.
    *   **Pros**: May provide more "complete" answers, potentially fewer explicit refusals, potentially handles questions where retrieved context is very sparse.
    *   **Cons**: Significantly increases the risk of hallucination and ungrounded responses, reduces the trustworthiness and verifiability of answers, directly violates a core requirement of this feature.
*   **Alternative 3: RAG with Few-shot Examples for Grounding**:
    *   **Description**: Provide the agent with few-shot examples in its prompt demonstrating correct grounding, refusal behavior, and source citation.
    *   **Pros**: Can be very effective in guiding agent behavior, complements system prompt instructions.
    *   **Cons**: Consumes more token context, which can be costly and hit context window limits for complex interactions. Test examples need careful curation.

## References

-   Feature Spec: D:\CODING\physicalAI-Humanoid-robotics\specs\005-rag-agent-backend\spec.md
-   Implementation Plan: D:\CODING\physicalAI-Humanoid-robotics\specs\005-rag-agent-backend\plan.md
-   Research: D:\CODING\physicalAI-Humanoid-robotics\specs\005-rag-agent-backend\research.md (R-001, R-002, R-003)
-   Related ADRs: None
-   Evaluator Evidence: None
