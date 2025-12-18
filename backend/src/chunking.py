from typing import List, Dict, Any
from langchain_text_splitters import RecursiveCharacterTextSplitter
import uuid

def chunk_text(text: str, page_metadata: Dict[str, Any]) -> List[Dict[str, Any]]:
    """
    Applies a semantic chunking strategy to the extracted text, associating each chunk
    with relevant metadata.
    
    Args:
        text: The full extracted text content from extract_text_from_url.
        page_metadata: Page-level metadata obtained from extract_text_from_url.
                       Expected to contain at least 'source_url', 'title', 'headings'.
                       'document_section' might also be present.
    Returns:
        A list of dictionaries, where each dictionary represents a chunk and includes
        its text and initial metadata fields.
    """
    if not text:
        return []

    # Define separators for markdown-like text
    # Prioritize larger structural elements
    separators = [
        "\n\n\n", # Triple newline for significant breaks
        "\n\n",   # Double newline for paragraph breaks
        "\n",     # Single newline for line breaks
        " ",      # Space for word breaks
        "",       # Character breaks (fallback)
    ]

    # Add markdown heading separators dynamically if headings are in metadata
    if "headings" in page_metadata and page_metadata["headings"]:
        # Prepend heading separators to prioritize splitting by headings
        # This is a heuristic; actual implementation may need more sophisticated parsing
        # For simplicity, we assume headings are followed by newlines and are distinct.
        # This will need refinement for true semantic heading-aware splitting
        pass # Will handle this in a more refined way with actual markdown parsing if needed


    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=1000,  # Optimal chunk size depends on embedding model and content
        chunk_overlap=200, # Overlap helps maintain context between chunks
        length_function=len, # Use character length for simplicity
        separators=separators,
    )

    raw_chunks = text_splitter.split_text(text)
    
    processed_chunks: List[Dict[str, Any]] = []
    for i, chunk_text_content in enumerate(raw_chunks):
        # Determine the closest heading for the chunk (simple heuristic)
        # This logic could be significantly more complex for accurate heading attribution
        closest_heading = None
        if "headings" in page_metadata:
            # For a real implementation, you'd find the heading *before* the chunk text position
            # For now, we just pick the first heading as a placeholder
            if page_metadata["headings"]:
                closest_heading = page_metadata["headings"][0] if i == 0 else None # Simple heuristic

        chunk_metadata = {
            "text": chunk_text_content,
            "source_url": page_metadata.get("source_url"),
            "page_title": page_metadata.get("title"),
            "document_section": page_metadata.get("document_section"),
            "heading": closest_heading, # This needs refinement for accuracy
            "chunk_id": str(uuid.uuid4()) # Unique ID for each chunk
        }
        processed_chunks.append(chunk_metadata)
        
    return processed_chunks

# Example usage
if __name__ == "__main__":
    sample_text = """
# Introduction to Docusaurus

Docusaurus is a static site generator that helps you build optimized websites quickly.

## Getting Started

To get started, install Node.js and then create a new project.

### Installation

```bash
npx create-docusaurus@latest my-website classic
```

This will create a new Docusaurus project named `my-website`.

## Configuration

You can configure your website in `docusaurus.config.js`.

    """
    sample_metadata = {
        "source_url": "https://example.com/docs/intro",
        "title": "Introduction",
        "document_section": "intro",
        "headings": ["Introduction to Docusaurus", "Getting Started", "Installation", "Configuration"]
    }

    chunks = chunk_text(sample_text, sample_metadata)
    for i, chunk in enumerate(chunks):
        print(f"--- Chunk {i+1} ---")
        print(f"Text: {chunk['text']}")
        print(f"Metadata: Source URL={chunk['source_url']}, Heading={chunk['heading']}, Chunk ID={chunk['chunk_id']}")
        print("-" * 20)

