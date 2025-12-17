import asyncio
import aiohttp
import json

async def test_rag_flow():
    base_url = "http://localhost:8000"

    print("Testing RAG Chatbot API...")

    # Test 1: Health check
    print("\n1. Testing health check...")
    async with aiohttp.ClientSession() as session:
        async with session.get(f"{base_url}/") as response:
            health = await response.json()
            print(f"Health check: {health}")

    # Test 2: Embed some sample text
    print("\n2. Testing embedding...")
    sample_text = """
    Artificial intelligence (AI) is intelligence demonstrated by machines, in contrast to the natural intelligence displayed by humans and animals.
    Leading AI textbooks define the field as the study of "intelligent agents": any device that perceives its environment and takes actions that maximize its chance of successfully achieving its goals.
    Colloquially, the term "artificial intelligence" is often used to describe machines that mimic "cognitive" functions that humans associate with the human mind, such as "learning" and "problem solving".
    As machines become increasingly capable, tasks considered to require "intelligence" are often removed from the definition of AI, a phenomenon known as the AI effect.
    A quip in Tesler's Theorem says "AI is whatever hasn't been done yet." For instance, optical character recognition is frequently excluded from things considered to be AI, having become a routine technology.
    """

    embed_payload = {
        "text": sample_text,
        "chapter_id": "test-chapter-ai-intro",
        "offset": 0
    }

    async with aiohttp.ClientSession() as session:
        async with session.post(f"{base_url}/embed", json=embed_payload) as response:
            if response.status == 200:
                embed_result = await response.json()
                print(f"Embedding result: {embed_result}")
            else:
                print(f"Embedding failed with status {response.status}")
                print(await response.text())

    # Test 3: Query the chat endpoint
    print("\n3. Testing chat query...")
    chat_payload = {
        "question": "What is artificial intelligence?",
        "session_id": "test-session-123"
    }

    async with aiohttp.ClientSession() as session:
        async with session.post(f"{base_url}/chat", json=chat_payload) as response:
            if response.status == 200:
                chat_result = await response.json()
                print(f"Chat result: {json.dumps(chat_result, indent=2)}")
            else:
                print(f"Chat query failed with status {response.status}")
                print(await response.text())

    # Test 4: Test selected-text-only mode
    print("\n4. Testing selected-text-only mode...")
    selected_text_payload = {
        "question": "What are cognitive functions?",
        "selected_text": "machines that mimic 'cognitive' functions that humans associate with the human mind, such as 'learning' and 'problem solving'",
        "session_id": "test-session-123"
    }

    async with aiohttp.ClientSession() as session:
        async with session.post(f"{base_url}/chat", json=selected_text_payload) as response:
            if response.status == 200:
                selected_result = await response.json()
                print(f"Selected text result: {json.dumps(selected_result, indent=2)}")
            else:
                print(f"Selected text query failed with status {response.status}")
                print(await response.text())

if __name__ == "__main__":
    print("Starting RAG flow test...")
    asyncio.run(test_rag_flow())
    print("\nRAG flow test completed!")