# SKILL.md

---
name: gemini-via-openai-agents
description: This skill configures the OpenAI Agents/ChatKit SDK to use Google's Gemini models by routing through the official OpenAI-compatible endpoint. Use this skill when you want to run agents with Gemini (e.g., gemini-1.5-flash or gemini-1.5-pro) instead of OpenAI models, leveraging Gemini's free tier or multimodal capabilities while keeping the same Agents SDK code.
---

# Gemini via OpenAI Agents SDK

## Overview
This skill enables the **OpenAI Agents/ChatKit SDK** to connect to Google's Gemini models using the official OpenAI-compatible API endpoint provided by Google. This allows you to use powerful Gemini models (faster, cheaper, or multimodal) without changing your agent code structure.

It is ideal for:
- Switching to Gemini's generous free tier
- Using multimodal capabilities (vision in supported models)
- Running RAG chatbots or agents cost-effectively

## Prerequisites
- A valid **Gemini API key** from [Google AI Studio](https://aistudio.google.com/app/apikey)
- Installed OpenAI Agents/ChatKit SDK (`pip install openai-agents` or latest version in your project)

## Configuration Process
Follow these steps exactly to route the Agents SDK to Gemini.

### Step 1: Create Custom OpenAI Client
Use `AsyncOpenAI` (or `OpenAI` for synchronous) with Gemini's endpoint:

```python
from agents import AsyncOpenAI

client = AsyncOpenAI(
    api_key="YOUR_GEMINI_API_KEY_HERE",  # e.g., "AIzaSy..."
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)
```

**Important Notes**:
- The trailing `/` in `base_url` is required.
- Do **not** use an OpenAI API key here.

### Step 2: Select Gemini Model
Wrap the model using `OpenAIChatCompletionsModel`:

```python
from agents import OpenAIChatCompletionsModel

model = OpenAIChatCompletionsModel(
    model="gemini-1.5-flash",  # or "gemini-1.5-pro", "gemini-1.5-flash-latest", etc.
    openai_client=client,
)
```

Available models (as of December 2025):
- `gemini-1.5-flash` (fast, cost-effective, multimodal)
- `gemini-1.5-pro` (higher capability)
- Check latest models at: https://ai.google.dev/gemini-api/docs/models

### Step 3: Configure RunConfig
Pass the custom model and client:

```python
from agents import RunConfig

config = RunConfig(
    model=model,
    model_provider=client,
    tracing_disabled=True  # Optional: disable tracing if not needed
)
```

### Step 4: Run Your Agent
Your existing agent code remains unchanged:

```python
from agents import Agent, Runner

agent = Agent(
    name="MyRAGAgent",
    instructions="You are a helpful assistant answering questions about the Physical AI course book.",
    # ... other params like tools, guardrails, etc.
)

result = Runner.run_sync(
    agent,
    "What are the hardware requirements for the course?",
    run_config=config
)

print(result.final_output)
```

## Full Example Template
```python
from agents import (
    Agent,
    AsyncOpenAI,
    OpenAIChatCompletionsModel,
    RunConfig,
    Runner,
)

# Step 1: Custom client for Gemini
client = AsyncOpenAI(
    api_key="YOUR_GEMINI_API_KEY",
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

# Step 2: Gemini model
model = OpenAIChatCompletionsModel(
    model="gemini-1.5-flash",
    openai_client=client,
)

# Step 3: Config
config = RunConfig(
    model=model,
    model_provider=client,
    tracing_disabled=True
)

# Your agent
agent = Agent(
    name="CourseBookAgent",
    instructions="Answer questions based only on the Physical AI & Humanoid Robotics course content.",
)

# Run
result = Runner.run_sync(agent, "User question here", run_config=config)
print(result.final_output)
```

## Common Issues & Fixes
- **Authentication error**: Ensure the API key is valid and has no regional/IP restrictions.
- **Model not found**: Use the exact model name (case-sensitive).
- **Rate limits**: Gemini free tier is generous, but monitor usage in Google AI Studio.
- **Multimodal inputs**: Gemini supports images natively—ensure your agent/tools pass them correctly.

## When to Use This Skill
- Reducing costs with Gemini's free tier
- Faster inference using Flash models
- Adding vision/multimodal capabilities to your RAG chatbot
- Testing alternative LLM backends without refactoring code

## Anti-Patterns
- Do **not** mix OpenAI and Gemini keys in the same client.
- Do **not** use the standard `OpenAI()` client without overriding `base_url`.
- Avoid hardcoding API keys—use environment variables (`os.getenv`) in production.

This skill keeps your OpenAI Agents/ChatKit workflow intact while powering it with Google's Gemini models!
