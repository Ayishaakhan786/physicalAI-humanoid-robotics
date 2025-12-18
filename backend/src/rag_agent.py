from typing import List, Dict, Any, Optional
from openai import OpenAI
from src.utils.config import Config
from src.agent_tools import get_retrieval_tool
import logging

# Configure basic logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class RAGAgent:
    def __init__(self, openai_api_key: str, qdrant_client: QdrantClient, cohere_api_key: str, collection_name: str):
        self.client = OpenAI(api_key=openai_api_key)
        self.model = "gpt-4o-mini"  # Or another suitable OpenAI model
        self.temperature = 0.5
        self.max_iterations = 10

        self.retrieval_tool_func = get_retrieval_tool(qdrant_client, cohere_api_key, collection_name)
        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "retrieve_documents",
                    "description": "Retrieves relevant document chunks from Qdrant based on a natural language query.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {
                                "type": "string",
                                "description": "The natural language query to search for relevant documents."
                            },
                            "top_k": {
                                "type": "integer",
                                "description": "The number of top relevant chunks to retrieve (default is 5).",
                                "default": 5
                            }
                        },
                        "required": ["query"]
                    }
                }
            }
        ]

        # System prompt to enforce grounding and refusal
        self.system_prompt = """
        You are a helpful assistant that answers questions STRICTLY based on the provided context.
        You have access to a tool named `retrieve_documents` to get relevant information.
        
        Follow these rules:
        1. ALWAYS use the `retrieve_documents` tool to get information relevant to the user's question.
        2. Answer the user's question ONLY with the information retrieved from the tool.
        3. If the retrieved information does NOT contain the answer, or is insufficient, explicitly state that you cannot answer based on the provided context. DO NOT use your general knowledge.
        4. Cite your sources by including the `source_url` and `chunk_id` for each piece of information you use in your answer.
        5. Keep your answers concise and to the point.
        """

    def generate_response(self, user_question: str) -> Dict[str, Any]:
        """
        Generates a response to a user question using RAG by orchestrating retrieval and LLM reasoning.
        """
        messages = [
            {"role": "system", "content": self.system_prompt},
            {"role": "user", "content": user_question}
        ]

        try:
            for i in range(self.max_iterations):
                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    tools=self.tools,
                    tool_choice="auto",
                    temperature=self.temperature
                )
                
                response_message = response.choices[0].message
                tool_calls = response_message.tool_calls

                if tool_calls:
                    # Step 2: Respond to the tool call
                    messages.append(response_message)
                    
                    for tool_call in tool_calls:
                        function_name = tool_call.function.name
                        function_args = tool_call.function.arguments # This is a JSON string
                        
                        try:
                            # Execute the tool
                            if function_name == "retrieve_documents":
                                args = json.loads(function_args)
                                tool_output = self.retrieval_tool_func(**args)
                            else:
                                tool_output = {"error": f"Unknown tool: {function_name}"}
                            
                            messages.append(
                                {"tool_call_id": tool_call.id, "role": "tool", "name": function_name, "content": json.dumps(tool_output)}
                            )
                        except Exception as tool_ex:
                            logging.error(f"Error executing tool {function_name}: {tool_ex}")
                            messages.append(
                                {"tool_call_id": tool_call.id, "role": "tool", "name": function_name, "content": json.dumps({"error": str(tool_ex)})}
                            )
                else:
                    # Agent has generated a final answer
                    answer_content = response_message.content
                    
                    # Extract sources from the messages (if any tool calls happened)
                    sources: List[Source] = []
                    for msg in messages:
                        if msg["role"] == "tool" and "content" in msg and "source" in msg["content"]:
                            try:
                                tool_output_data = json.loads(msg["content"])
                                for item in tool_output_data:
                                    if "source" in item:
                                        # Assuming the source object in tool_output is compatible with our Source model
                                        sources.append(Source(**item["source"]))
                            except json.JSONDecodeError:
                                pass # Malformed tool output

                    return {"answer": answer_content, "sources": sources}

            logging.warning(f"Agent reached max iterations ({self.max_iterations}) without a final answer.")
            return {"answer": "I could not generate a complete answer within the allowed iterations.", "sources": []}

        except Exception as e:
            logging.error(f"Error during RAG agent processing: {e}")
            return {"answer": f"An error occurred: {e}", "sources": []}

# Example usage
if __name__ == "__main__":
    import json
    from qdrant_client import QdrantClient
    
    try:
        Config.validate()
    except ValueError as e:
        logging.error(f"Configuration Error: {e}")
        exit(1)

    openai_api_key = Config.OPENAI_API_KEY
    cohere_api_key = Config.COHERE_API_KEY
    qdrant_url = Config.QDRANT_URL
    qdrant_api_key = Config.QDRANT_API_KEY
    collection_name = "rag_embedding"

    qdrant_client_instance = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

    rag_agent = RAGAgent(openai_api_key, qdrant_client_instance, cohere_api_key, collection_name)

    print("\n--- Testing RAG Agent with In-scope Question ---")
    in_scope_question = "What are the core components of ROS2?"
    response = rag_agent.generate_response(in_scope_question)
    print(json.dumps(response, indent=2))

    print("\n--- Testing RAG Agent with Out-of-scope Question ---")
    out_of_scope_question = "What is the capital of France?"
    response = rag_agent.generate_response(out_of_scope_question)
    print(json.dumps(response, indent=2))