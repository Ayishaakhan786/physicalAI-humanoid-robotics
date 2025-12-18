// book/src/services/api.js

const API_BASE_URL = process.env.RAG_BACKEND_URL || 'http://localhost:8000';

export async function postQuery(queryData) {
  try {
    const response = await fetch(`${API_BASE_URL}/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(queryData),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    console.error("Error in postQuery:", error);
    throw error;
  }
}