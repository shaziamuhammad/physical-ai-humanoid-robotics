#!/usr/bin/env python3
"""
Check Qdrant QueryResponse object structure
"""

import qdrant_client
import os
from qdrant_client.http import models
import uuid
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize Qdrant client
qdrant = qdrant_client.QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

COLLECTION_NAME = "book_embeddings"

def check_query_response_structure():
    print("Testing Qdrant query_points to see response structure...")

    # Create a simple test query vector (using a simple 4-dimensional vector for testing)
    test_query = [0.1, 0.2, 0.3, 0.4]  # This is just to test the structure

    try:
        # Try query_points with minimal parameters to see what attributes are available
        result = qdrant.query_points(
            collection_name=COLLECTION_NAME,
            query=test_query,
            limit=1
        )

        print(f"QueryResponse type: {type(result)}")
        print(f"Available attributes: {[attr for attr in dir(result) if not attr.startswith('_')]}")

        # Print the actual object to see its representation
        print(f"QueryResponse object: {result}")

        # Try different attribute names that might exist
        possible_attrs = ['results', 'points', 'hits', 'data', 'payloads', 'documents']
        for attr in possible_attrs:
            if hasattr(result, attr):
                print(f"Attribute '{attr}' exists: {getattr(result, attr)}")
            else:
                print(f"Attribute '{attr}' does not exist")

    except Exception as e:
        print(f"Error during query: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    check_query_response_structure()