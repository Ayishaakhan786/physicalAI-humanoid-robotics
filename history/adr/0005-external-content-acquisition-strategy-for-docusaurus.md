# ADR-0005: External Content Acquisition Strategy for Docusaurus

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

-   **Status:** Proposed
-   **Date:** 2025-12-18
-   **Feature:** 003-rag-content-ingestion
-   **Context:** The RAG pipeline requires ingesting content from a deployed Docusaurus website. The choice of strategy for acquiring this content is critical, as it impacts robustness, reliability, and the quality of the extracted text. This decision covers how URLs will be discovered and how the actual page content will be fetched and prepared for chunking and embedding, taking into account the nature of a Docusaurus-generated static site.

## Decision

The content acquisition process will utilize standard Python libraries for web scraping. Specifically, the `requests` library will be used to fetch HTML content from the deployed Docusaurus website URLs. URL discovery will primarily involve parsing the Docusaurus sitemap (e.g., `sitemap.xml`) to identify all public documentation pages. Once HTML is retrieved, the `BeautifulSoup` library will be employed to parse the HTML structure and extract clean, structured textual content, with a focus on identifying and isolating the main content areas, while filtering out navigation elements, headers, footers, sidebars, and other boilerplate.

## Consequences

### Positive

*   **Direct Access to Live Content**: Ensures that the pipeline is always working with the most up-to-date version of the deployed documentation, reflecting any recent changes.
*   **Robust HTML Parsing**: `BeautifulSoup` provides a flexible and powerful way to navigate and extract data from complex HTML structures, allowing for targeted content extraction based on CSS selectors or HTML element attributes.
*   **Standard Python Libraries**: Relies on widely used, well-documented, and stable Python libraries (`requests`, `BeautifulSoup`), which have extensive community support and are familiar to many Python developers.
*   **No Direct Repository Access Needed**: This strategy does not require access to the Docusaurus source code repository, simplifying deployment and access management, especially for publicly available sites.

### Negative

*   **Fragility to UI Changes**: The scraping logic (e.g., `BeautifulSoup` selectors, parsing heuristics) is highly dependent on the Docusaurus website's specific HTML structure and CSS classes. Changes to the Docusaurus theme, plugins, or rendering logic could break the extraction process, requiring frequent maintenance and updates to the scraping code.
*   **Network Dependence**: The pipeline's performance and reliability are directly subject to network latency, the availability of the deployed Docusaurus website, and the efficiency of the web server.
*   **Sitemap Reliability**: Dependence on `sitemap.xml` assumes it is always present, up-to-date, and accurately lists all public URLs relevant for ingestion. Issues with the sitemap could lead to incomplete data ingestion.
*   **Rate Limiting Risks**: Aggressive crawling (fetching many pages too quickly) might trigger rate limits on the Docusaurus site's hosting provider, potentially leading to temporary IP bans or blocking of the crawler.
*   **Client-Side Rendering Issues**: While Docusaurus primarily generates static HTML, if any significant content relies on client-side JavaScript for rendering, `requests` alone (which fetches raw HTML) might not capture the full, dynamically loaded content. This would necessitate more advanced and complex browser automation tools.

## Alternatives Considered

*   **Alternative 1: Direct Markdown Source Ingestion**:
    *   **Description**: Acquire the raw Markdown source files directly from the Docusaurus project's source code repository (e.g., a Git clone, API access to GitHub/GitLab). 
    *   **Pros**: Highly robust to UI changes, provides the cleanest and most semantic source content directly, avoids network latency and the fragility of web scraping. Ensures all content (including drafts not yet published) is available.
    *   **Cons**: Requires access to the Docusaurus source code repository (which may not always be available for a public site), might involve complexities with build processes or specific Markdown dialect parsing, and fundamentally changes the scope from a deployed URL to source files.
*   **Alternative 2: Docusaurus API/Plugin (if available)**:
    *   **Description**: Investigate if Docusaurus offers a headless content API or a plugin that programmatically exposes content in a structured format (e.g., JSON, GraphQL).
    *   **Pros**: The most robust and officially supported method if available, provides structured data directly, avoids scraping fragility and its associated maintenance burden.
    *   **Cons**: A dedicated Docusaurus content API might not exist out-of-the-box, or would require custom development of a specific plugin, increasing dependency on Docusaurus's internal architecture.
*   **Alternative 3: Browser Automation (e.g., Selenium/Playwright)**:
    *   **Description**: Use browser automation tools (like Selenium or Playwright) to programmatically browse the Docusaurus site, render pages, and extract content. This method is capable of handling client-side rendered content.
    *   **Pros**: Can handle complex JavaScript-rendered content, more robust to certain types of dynamic page content compared to raw `requests`.
    *   **Cons**: Significantly higher computational and resource overhead (requires running a headless browser), much slower execution compared to direct HTTP requests, more complex to set up and maintain, and introduces a heavier browser dependency.

## References

-   Feature Spec: D:\CODING\physicalAI-Humanoid-robotics\specs\003-rag-content-ingestion\spec.md
-   Implementation Plan: D:\CODING\physicalAI-Humanoid-robotics\specs\003-rag-content-ingestion\plan.md
-   Research: D:\CODING\physicalAI-Humanoid-robotics\specs\003-rag-content-ingestion\research.md (R-001)
-   Related ADRs: None
-   Evaluator Evidence: None
