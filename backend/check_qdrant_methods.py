#!/usr/bin/env python3
"""
Test script to check Qdrant client methods
"""

import qdrant_client
import os

# Initialize Qdrant client
qdrant = qdrant_client.QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

print("Available methods on qdrant client:")
methods = [method for method in dir(qdrant) if not method.startswith('_')]
for method in sorted(methods):
    if not method.startswith('_'):
        print(f"  {method}")

print("\nChecking if 'search' method exists:")
print(f"Has search: {hasattr(qdrant, 'search')}")

print("\nChecking if 'search_points' method exists:")
print(f"Has search_points: {hasattr(qdrant, 'search_points')}")