#!/usr/bin/env python3
"""
Test script to verify Qdrant search functionality
"""

import sys
import os

# Add the backend directory to the path so we can import modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from embedding import search_embeddings
import logging

# Configure logging to show detailed information
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

def test_search():
    print("Testing Qdrant search functionality...")
    try:
        # Test with a simple query
        result = search_embeddings("What is ROS2?", limit=3, score_threshold=0.0)  # Lower threshold to find any matches
        print(f"Search successful! Found {len(result['relevant_texts'])} relevant texts")
        print(f"Citations: {len(result['citations'])}")
        if result['citations']:
            print(f"First citation chapter: {result['citations'][0]['chapter_id']}")
            print(f"First citation score: {result['citations'][0]['score']}")
    except Exception as e:
        print(f"Error during search: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_search()