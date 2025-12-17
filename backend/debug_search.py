#!/usr/bin/env python3
"""
Debug script to see the structure of Qdrant search results
"""

import sys
import os

# Add the backend directory to the path so we can import modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from embedding import search_embeddings
import logging

# Configure logging to show detailed information
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

def debug_search():
    print("Debugging Qdrant search functionality...")
    try:
        # Test with a simple query
        result = search_embeddings("robotics", limit=3, score_threshold=0.0)  # Search for "robotics" which should be in the book
        print(f"Search successful! Found {len(result['relevant_texts'])} relevant texts")
        print(f"Citations: {len(result['citations'])}")
        if result['citations']:
            print(f"First citation chapter: {result['citations'][0]['chapter_id']}")
            print(f"First citation score: {result['citations'][0]['score']}")
            print(f"First citation text preview: {result['citations'][0]['text'][:100]}...")
        else:
            print("No citations found, but search didn't error out")
    except Exception as e:
        print(f"Error during search: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_search()