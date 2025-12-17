import requests
import json

def test_api():
    base_url = "http://localhost:8000"

    print("Testing API endpoints...")

    # Test health check
    try:
        response = requests.get(f"{base_url}/")
        print(f"Health check: {response.json()}")
    except Exception as e:
        print(f"Health check failed: {e}")
        return

    # Test with a clean, valid selected text
    test_payload = {
        "question": "What is artificial intelligence?",
        "selected_text": "Artificial intelligence (AI) is intelligence demonstrated by machines, in contrast to the natural intelligence displayed by humans and animals.",
        "session_id": "test-session-123"
    }

    print(f"\nTesting with payload: {json.dumps(test_payload, indent=2)}")

    try:
        response = requests.post(f"{base_url}/chat", json=test_payload)
        print(f"Response status: {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            print(f"Response: {json.dumps(result, indent=2)}")
        else:
            print(f"Error response: {response.text}")
    except Exception as e:
        print(f"Request failed: {e}")

if __name__ == "__main__":
    test_api()