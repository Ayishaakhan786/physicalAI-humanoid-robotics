import os
from dotenv import load_dotenv

load_dotenv() # Load environment variables from .env file

class Config:
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    DOCUSAURUS_BASE_URL: str = os.getenv("DOCUSAURUS_BASE_URL", "")

    @classmethod
    def validate(cls):
        missing_vars = [name for name, value in cls.__dict__.items() if isinstance(value, str) and not value]
        if missing_vars:
            raise ValueError(f"Missing required environment variables: {', '.join(missing_vars)}")

# Example usage:
# if __name__ == "__main__":
#     try:
#         Config.validate()
#         print("All environment variables loaded and validated.")
#         print(f"Cohere API Key: {Config.COHERE_API_KEY[:5]}...") # print first 5 chars
#     except ValueError as e:
#         print(f"Configuration Error: {e}")
