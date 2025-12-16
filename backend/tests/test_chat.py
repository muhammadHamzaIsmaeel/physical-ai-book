# backend/tests/test_chat.py
from fastapi.testclient import TestClient
from ..src.main import app

client = TestClient(app)

def test_chat_endpoint_streaming():
    """
    A simple smoke test to ensure the /chat endpoint returns a streaming response.
    This does not validate the content of the stream, only that it behaves like a stream.
    """
    response = client.post(
        "/chat",
        json={"message": "What is URDF?"},
        stream=True  # Use stream=True to test streaming responses
    )
    
    assert response.status_code == 200
    assert "text/event-stream" in response.headers["content-type"]
    
    # Check that we receive some data and the stream closes
    lines = []
    for line in response.iter_lines():
        lines.append(line)
    
    assert len(lines) > 0
    # The last line should be the [DONE] signal
    assert lines[-1] == "data: [DONE]"

def test_read_root():
    """ Test the root endpoint. """
    response = client.get("/")
    assert response.status_code == 200
    assert response.json() == {"message": "RAG Chatbot API is running."}
