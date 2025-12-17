# Docusaurus Theming Research

**Date**: 2025-12-17
**Feature**: [Upgrade Docusaurus Book UI/UX](spec.md)

## Research Question

What is the most effective and maintainable strategy for implementing a significant UI/UX overhaul in a Docusaurus project, covering typography, colors, spacing, and component structure?

## Options Considered

1.  **Infima CSS Variables (`src/css/custom.css`)**
    *   **Description**: Docusaurus is built on the Infima CSS framework. The primary method for theming is to override its root CSS variables.
    *   **Pros**:
        *   Official, recommended, and simplest method for theming.
        *   Good for global changes to color, typography, spacing, and border-radius.
        *   Low risk of breaking changes during Docusaurus upgrades.
    *   **Cons**:
        *   Limited to the variables exposed by Infima. Cannot be used for structural changes to components (e.g., rearranging elements in the Navbar).

2.  **Docusaurus Swizzling**
    *   **Description**: A feature that allows developers to eject a theme component and replace it with a custom implementation.
    *   **Pros**:
        *   Provides full control over the component's structure, style, and logic.
        *   Necessary for any changes that go beyond what Infima variables allow.
    *   **Cons**:
        *   Creates a maintenance burden. The swizzled component is detached from theme updates, so future Docusaurus upgrades may require manual updates to the custom component.
        *   Can be complex. Requires understanding the internal structure of the theme components.

3.  **Tailwind CSS Integration**
    *   **Description**: Integrating the Tailwind CSS utility-first framework into the Docusaurus build process.
    *   **Pros**:
        *   Allows for rapid development of custom UIs without writing custom CSS.
        *   Excellent for creating unique, non-standard layouts.
    *   **Cons**:
        *   Adds complexity to the build setup (requires PostCSS).
        *   Can conflict with Infima's default styles, requiring careful style scoping and potentially disabling parts of Infima.
        *   Increases the maintenance overhead and learning curve for new developers on the project.

## Decision & Rationale

**Decision**: A hybrid approach is recommended.

1.  **Primary Strategy (Infima CSS Variables)**: All global style changes (colors, fonts, spacing, etc.) will be implemented by overriding Infima's CSS variables in `src/css/custom.css`. This is the cleanest and most maintainable approach and should be used for the majority of the work.

2.  **Secondary Strategy (Swizzling)**: When a component requires structural changes that cannot be achieved with CSS alone (e.g., altering the layout of the footer or adding new elements to the navbar), we will use "swizzling". This should be used sparingly to minimize maintenance overhead.

3.  **Avoidance (Tailwind CSS)**: We will **not** integrate Tailwind CSS at this time. The complexity of integrating it and the potential for conflicts with the base theme outweigh the benefits for this project, which is an enhancement of an existing structure, not a from-scratch build.

This tiered approach prioritizes maintainability and aligns with Docusaurus best practices, while still providing the power to make deeper changes where necessary.
