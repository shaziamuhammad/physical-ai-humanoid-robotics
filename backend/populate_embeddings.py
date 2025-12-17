#!/usr/bin/env python3
"""
Script to populate Qdrant with book embeddings from the sitemap
"""

import asyncio
import sys
import os

# Add the backend directory to the path so we can import modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from embedding import populate_book_embeddings, initialize_collection
import logging

# Configure logging to show detailed information
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

def main():
    print("Initializing Qdrant collection...")
    initialize_collection()

    print("Starting to populate book embeddings from sitemap...")
    try:
        result = populate_book_embeddings()
        print(f"Successfully populated book embeddings: {result}")
    except Exception as e:
        print(f"Error populating book embeddings: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()