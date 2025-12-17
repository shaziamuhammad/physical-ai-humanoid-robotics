#!/usr/bin/env python3
"""
Google Gemini API Key and Connectivity Verification Script
"""
import os
import google.generativeai as genai
import logging
import traceback
from dotenv import load_dotenv

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def mask_api_key(api_key):
    """Mask the middle characters of the API key for safety"""
    if not api_key or len(api_key) < 8:
        return api_key
    return api_key[:4] + '*' * (len(api_key) - 8) + api_key[-4:]

def verify_gemini_connectivity():
    """
    CCR: Verify Google Gemini API key and connectivity with full diagnostics
    """
    try:
        logger.info("Starting Google Gemini API key and connectivity verification...")

        # Load environment variables
        load_dotenv()

        # Read the current .env file and confirm the GEMINI_API_KEY value
        api_key = os.getenv("GEMINI_API_KEY")

        if not api_key:
            logger.error("GEMINI_API_KEY not found in environment variables")
            return {
                "key_loaded_correctly": False,
                "api_call_success": False,
                "error_message": "GEMINI_API_KEY environment variable is not set",
                "response_preview": None
            }

        logger.info(f"API key loaded: {mask_api_key(api_key)}")

        # Configure the Gemini client
        genai.configure(api_key=api_key)

        # Initialize the model
        model = genai.GenerativeModel('gemini-flash-latest')  # Using the latest flash model available

        # Attempt a minimal test call to Gemini API
        logger.info("Attempting test call to Google Gemini API...")
        response = model.generate_content(
            "Hello, please respond with a simple greeting.",
            generation_config={
                "temperature": 0.0,  # Deterministic response
                "max_output_tokens": 20,  # Minimal response
            }
        )

        # If we get here, the API call was successful
        response_content = response.text if response.text else "No response text received"
        logger.info(f"Gemini API call successful! Response preview: {response_content[:100]}")

        return {
            "key_loaded_correctly": True,
            "api_call_success": True,
            "error_message": None,
            "response_preview": response_content[:100] if response_content else None
        }

    except Exception as e:
        # Catch any exceptions during Gemini API call
        logger.error(f"Google Gemini API Error: {str(e)}")
        logger.error(f"Full traceback:\n{traceback.format_exc()}")

        # Check for specific error types
        error_message = str(e)
        if "API key not valid" in error_message or "400" in error_message or "401" in error_message:
            return {
                "key_loaded_correctly": True,  # Key exists but is invalid
                "api_call_success": False,
                "error_message": f"Authentication failed - Invalid API key: {str(e)}",
                "response_preview": None
            }
        else:
            return {
                "key_loaded_correctly": api_key is not None,
                "api_call_success": False,
                "error_message": f"Gemini API error: {str(e)}",
                "response_preview": None
            }

def main():
    """Main function to run the Gemini verification"""
    logger.info("Starting Google Gemini API verification process...")

    result = verify_gemini_connectivity()

    # Print the JSON report
    print("\n" + "="*50)
    print("GEMINI API VERIFICATION REPORT")
    print("="*50)
    print(f"Key loaded correctly: {result['key_loaded_correctly']}")
    print(f"API call success: {result['api_call_success']}")
    if result['error_message']:
        print(f"Error message: {result['error_message']}")
    if result['response_preview']:
        print(f"Response preview: {result['response_preview']}")
    print("="*50)

    # Return the result for programmatic use
    return result

if __name__ == "__main__":
    main()