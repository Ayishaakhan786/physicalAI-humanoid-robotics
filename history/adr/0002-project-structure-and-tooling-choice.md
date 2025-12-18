# ADR-0002: Project Structure and Tooling Choice

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

-   **Status:** Proposed
-   **Date:** 2025-12-18
-   **Feature:** 003-rag-content-ingestion
-   **Context:** The initial implementation of the RAG content ingestion pipeline requires defining the foundational project structure and the core tooling for development and dependency management. Given the requirements for a Python-based solution and the need for a focused, runnable script, decisions regarding the programming language, environment manager, and main script organization are critical for development workflow and future maintainability.

## Decision

The project will be structured using Python as the primary programming language. Dependency and environment management will be handled by `uv`. The core logic for the RAG content ingestion pipeline will reside within a single `main.py` file, placed within a dedicated `backend/` directory at the project root.

## Consequences

### Positive

*   **Simplicity**: A single `main.py` file simplifies development and execution for this focused pipeline, especially for a prototype or initial iteration.
*   **Modern Dependency Management**: `uv` provides fast and reliable dependency resolution and environment setup, improving developer experience and build times.
*   **Clarity**: Isolating backend logic in a `backend/` directory clearly separates it from other potential project components (e.g., frontend, documentation), enhancing project organization.
*   **Python Constraint Met**: Adheres to the explicit project constraint for using Python as the implementation language.

### Negative

*   **Scalability Limitations (Single File)**: A single `main.py` might become unwieldy and difficult to navigate for larger, more complex pipelines or when integrating more functionalities, potentially leading to reduced modularity and increased cognitive load if not refactored later.
*   **Tooling Lock-in**: Relying on `uv` creates a dependency on a specific tool, which, while modern and performant, introduces an external dependency that would need to be considered if switching environments or toolchains.
*   **Potential for Feature Creep**: The perceived simplicity of a single `main.py` could inadvertently encourage adding unrelated logic, blurring responsibilities if strict boundaries are not enforced.

## Alternatives Considered

*   **Alternative 1: Traditional `venv` + `pip`**:
    *   **Description**: Use Python's built-in `venv` module for environment isolation and the `pip` package installer for dependency management.
    *   **Pros**: Widely adopted, requires minimal overhead to set up, no external tool dependency beyond Python itself.
    *   **Cons**: Generally slower dependency resolution and installation compared to `uv`, less advanced dependency locking and resolution features than more modern tools like `uv` or `Poetry`.
*   **Alternative 2: Multi-file Python Package**:
    *   **Description**: Organize the pipeline as a proper Python package with multiple modules (e.g., `src/ingestion/`, `src/embedding/`, `src/qdrant/`), utilizing `__init__.py` files and other standard package conventions.
    *   **Pros**: Enhanced modularity, better scalability for complex logic, clear separation of concerns, easier testing of individual components.
    *   **Cons**: More upfront setup complexity, might be considered overkill for a simple runnable script as initially requested, requiring more files and potentially a `setup.py` or `pyproject.toml` for package definition.

## References

-   Feature Spec: D:\CODING\physicalAI-Humanoid-robotics\specs\003-rag-content-ingestion\spec.md
-   Implementation Plan: D:\CODING\physicalAI-Humanoid-robotics\specs\003-rag-content-ingestion\plan.md
-   Related ADRs: None
-   Evaluator Evidence: None
