# Data Model: RAG Content Ingestion and Vector Storage

**Feature**: 003-rag-content-ingestion
**Date**: 2025-12-18

## Overview

This document describes the key data entities involved in the RAG content ingestion and vector storage pipeline, outlining their structure, attributes, and relationships. These entities directly correspond to the `Key Entities` identified in the feature specification and further elaborated in the implementation plan.

## Entities

### 1. DocusaurusPage

Represents a single page extracted from the deployed Docusaurus website.

-   **`url`** (string, **Primary Key**): The absolute and unique URL of the Docusaurus page (e.g., `https://physical-ai-humanoid-robotics-umber.vercel.app/docs/intro`).
-   **`html_content`** (string, optional): The raw HTML content fetched from the URL. This is an intermediate, transient attribute for text extraction.
-   **`extracted_text`** (string): The clean, structured textual content extracted from the `html_content`. This includes markdown, headings, paragraphs, code blocks, etc., without navigation or boilerplate.
-   **`title`** (string, optional): The main title of the Docusaurus page, extracted from HTML (e.g., `<title>` tag or `<h1>`).
-   **`language`** (string, optional): The language of the content, if identifiable (e.g., 'en').
-   **`last_modified_date`** (datetime, optional): The timestamp indicating when the page was last modified, potentially extracted from the sitemap or HTTP headers. Useful for change detection.

### 2. TextChunk

Represents a semantically coherent segment of a `DocusaurusPage`'s `extracted_text`, suitable for embedding.

-   **`id`** (string, **Primary Key**): A unique identifier for the chunk. This could be a hash of its content and `source_url` to ensure uniqueness and detect duplicates (e.g., SHA256 hash).
-   **`text`** (string): The actual textual content of the chunk.
-   **`source_url`** (string, **Foreign Key to DocusaurusPage.url**): The URL of the `DocusaurusPage` from which this chunk was derived.
-   **`token_count`** (int): The number of tokens in the chunk's `text`, as determined by the Cohere model's tokenizer or an approximate method.
-   **`char_start_index`** (int): The starting character index of the chunk within its parent `DocusaurusPage.extracted_text`.
-   **`char_end_index`** (int): The ending character index of the chunk within its parent `DocusaurusPage.extracted_text`.
-   **`page_title`** (string, optional): The title of the parent Docusaurus page.
-   **`document_section`** (string, optional): A categorisation of the content within the overall book structure (e.g., 'module-01-ros2', 'capstone-appendix'). This could be inferred from URL paths or page metadata.
-   **`heading`** (string, optional): The closest major heading (`<h1>` to `<h3>`) preceding the chunk within its `source_url`'s content.

### 3. VectorEmbedding

Represents the numerical vector embedding generated from a `TextChunk`.

-   **`chunk_id`** (string, **Primary Key**, **Foreign Key to TextChunk.id**): The unique identifier of the `TextChunk` that this vector represents.
-   **`vector`** (list of float): The dense numerical vector embedding itself.
-   **`dimensionality`** (int): The dimension of the vector (e.g., 1024 for a Cohere model).
-   **`model_name`** (string): The identifier of the Cohere model used to generate this embedding (e.g., `embed-english-v3.0`).
-   **`generated_at`** (datetime): Timestamp of when this embedding was generated.

### 4. EmbeddingMetadata (Qdrant Payload)

This entity describes the structured data associated with each `VectorEmbedding` when stored in Qdrant. It is essentially the payload for each vector point in Qdrant.

-   **`source_url`** (string, **Indexed**): The URL of the original Docusaurus page.
-   **`document_section`** (string, optional, **Indexed**): The inferred section of the document.
-   **`heading`** (string, optional, **Indexed**): The closest parent heading to the chunk.
-   **`chunk_id`** (string, **Indexed**): Unique identifier for the `TextChunk`.
-   **`chunk_text_preview`** (string): A short, truncated version of the `TextChunk.text` for display/debugging purposes (e.g., first 200 characters).
-   **`page_title`** (string, optional, **Indexed**): The title of the Docusaurus page.
-   **`last_modified_date`** (datetime, optional, **Indexed**): Last modification date of the source page.
-   **`updated_at`** (datetime): Timestamp of when this embedding and metadata record was last stored/updated in Qdrant.

### 5. QdrantCollection

Represents the configuration and state of the vector collection in Qdrant Cloud.

-   **`name`** (string, **Primary Key**): `rag_embedding`.
-   **`vector_size`** (int): The expected dimensionality of vectors to be stored (must match `VectorEmbedding.dimensionality`).
-   **`distance_metric`** (string): The similarity metric used for search (e.g., 'Cosine', 'Dot', 'Euclid').
-   **`points_count`** (int): The current number of vector points stored in the collection.
-   **`created_at`** (datetime): Timestamp of collection creation.
-   **`last_updated_at`** (datetime): Timestamp of last significant update to the collection (e.g., new vectors added).

## Relationships

-   **One-to-Many**: A `DocusaurusPage` can generate multiple `TextChunk`s.
-   **One-to-One**: Each `TextChunk` corresponds to exactly one `VectorEmbedding`.
-   **One-to-One**: Each `VectorEmbedding` is stored in Qdrant with its corresponding `EmbeddingMetadata` (as payload).
-   **Many-to-One**: Many `VectorEmbedding`s (and their `EmbeddingMetadata`) are stored within a single `QdrantCollection`.

## Validation Rules (Examples)

-   `DocusaurusPage.url`: Must be a valid HTTP/HTTPS URL.
-   `TextChunk.text`: Cannot be empty after extraction.
-   `VectorEmbedding.vector`: Must match `QdrantCollection.vector_size`.
-   `EmbeddingMetadata.source_url`: Must be a valid URL corresponding to an ingested page.
-   All `EmbeddingMetadata` fields marked as **Indexed** should be configured as such in Qdrant for efficient filtering.