import logging
import time
from functools import wraps
from requests import RequestException
from qdrant_client.http.exceptions import UnexpectedResponse, ResponseHandlingException

# Note: CohereAPIError is currently caught by general Exception in main.py
# If a specific Cohere error needs to be caught, it should be added here
# from cohere import CohereAPIError

def retry(exceptions, tries=4, delay=3, backoff=2):
    """
    Retry decorator with exponential backoff.
    :param exceptions: An exception or tuple of exceptions to catch.
    :param tries: Number of times to try (not including the first attempt).
    :param delay: Initial delay between retries in seconds.
    :param backoff: Multiplier for delay between retries.
    """
    def deco_retry(f):
        @wraps(f)
        def f_retry(*args, **kwargs):
            mtries, mdelay = tries, delay
            while mtries > 1:
                try:
                    return f(*args, **kwargs)
                except exceptions as e:
                    logging.warning(f"{e}, Retrying in {mdelay} seconds...")
                    time.sleep(mdelay)
                    mtries -= 1
                    mdelay *= backoff
            return f(*args, **kwargs) # Last try without catching
        return f_retry
    return deco_retry