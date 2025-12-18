import requests
from typing import List, Optional, Tuple, Dict, Any
from urllib.parse import urljoin
from bs4 import BeautifulSoup

def get_all_urls(base_url: str) -> List[str]:
    """
    Discovers all public documentation URLs from the specified Docusaurus base_url
    by parsing its sitemap.xml.
    """
    sitemap_url = urljoin(base_url, "sitemap.xml")
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status() # Raise an HTTPError for bad responses (4xx or 5xx)
        soup = BeautifulSoup(response.content, 'xml')
        urls = []
        for loc in soup.find_all('loc'):
            original_url = loc.text
            # Replace placeholder hostname if it exists
            if "your-docusaurus-site.example.com" in original_url:
                corrected_url = original_url.replace("https://your-docusaurus-site.example.com", base_url)
                urls.append(corrected_url)
            else:
                urls.append(original_url)
        # Filter for relevant documentation URLs if necessary, e.g., excluding blog posts
        return [url for url in urls if "/docs/" in url]
    except requests.exceptions.RequestException as e:
        print(f"Error fetching sitemap from {sitemap_url}: {e}")
        raise
    except Exception as e:
        print(f"Error parsing sitemap from {sitemap_url}: {e}")
        raise

import re
def extract_text_from_url(url: str) -> Optional[Tuple[str, Dict[str, Any]]]:
    """
    Fetches the HTML content of a given Docusaurus page and extracts clean, structured text
    from the main content area along with relevant page-level metadata.
    """
    try:
        response = requests.get(url)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, 'html.parser')

        page_metadata = {
            "source_url": url,
            "title": soup.title.string if soup.title else "Untitled",
            "document_section": None,
            "last_modified_date": None # Can be inferred from response headers if available, or sitemap.xml
        }

        # Infer document_section from URL path
        path_parts = [part for part in url.split('/') if part]
        if "docs" in path_parts:
            try:
                docs_index = path_parts.index("docs")
                if len(path_parts) > docs_index + 1:
                    page_metadata["document_section"] = path_parts[docs_index + 1] # More robust heuristic
            except ValueError:
                pass # "docs" not in path_parts, already handled

        # Docusaurus content is typically within a div with specific classes
        # This might need adjustment based on the specific Docusaurus theme/version
        # Targeting common Docusaurus content selectors
        content_div = soup.find('main', class_='DocItem_content_OLP1') or \
                      soup.find('div', class_='markdown') or \
                      soup.find('div', class_=re.compile(r'docItemContainer')) or \
                      soup.find('article') # Fallback for general article content

        if content_div:
            # Remove script, style, nav, header, footer elements
            for script_or_style in content_div(['script', 'style', 'nav', 'header', 'footer']):
                script_or_style.decompose()
            
            # Extract main headings for metadata
            headings = content_div.find_all(['h1', 'h2', 'h3'])
            if headings:
                page_metadata["headings"] = [h.get_text(strip=True) for h in headings]
            
            # Get text and clean up excessive newlines/spaces
            text = content_div.get_text(separator='\n', strip=True)
            return text, page_metadata
        else:
            print(f"Warning: Could not find main content div for {url}")
            return None, page_metadata # Return metadata even if no text
    except requests.exceptions.RequestException as e:
        print(f"Error fetching content from {url}: {e}")
        return None, {"source_url": url, "title": "Error", "document_section": None, "last_modified_date": None}
    except Exception as e:
        print(f"Error parsing content from {url}: {e}")
        return None, {"source_url": url, "title": "Error", "document_section": None, "last_modified_date": None}

# Example usage (for testing functions)
if __name__ == "__main__":
    base_url = "https://physical-ai-humanoid-robotics-umber.vercel.app/"
    try:
        print(f"Fetching URLs from {base_url}...")
        urls = get_all_urls(base_url)
        print(f"Found {len(urls)} URLs.")
        if urls:
            for i, url in enumerate(urls[:5]): # Print first 5 for brevity
                print(f"\nExtracting text from: {url}")
                text = extract_text_from_url(url)
                if text:
                    print(f"Extracted text length: {len(text)} chars (first 200):\n{text[:200]}...")
                else:
                    print("No text extracted.")
                if i >= 4: break
    except Exception as e:
        print(f"An error occurred during example usage: {e}")
