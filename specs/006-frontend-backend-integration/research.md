# Research for Frontend and Backend Integration

This document outlines the research and decisions made for integrating the RAG chatbot with the Docusaurus frontend.

## T001: Docusaurus Integration Method

**Decision**: We will use a custom React component injected into the Docusaurus layout by "swizzling" the theme.

**Rationale**:
- **Swizzling**: Docusaurus's swizzling feature allows overriding default theme components with custom implementations. By swizzling the main `Layout` component, we can inject our `ChatbotUI` component so it appears on every page.
- **Custom Plugin**: A custom plugin would be more complex to create and maintain for the simple purpose of adding a single component.
- **Theme Swizzling**: This is the recommended approach by Docusaurus for this kind of customization. It's powerful and flexible.

## T002: Capturing User-Selected Text

**Decision**: We will use `window.getSelection().toString()` to capture user-selected text.

**Rationale**:
- This is a standard browser API and works reliably across all modern browsers.
- It's simple to implement and does not require any external libraries.

## T003: CORS and Environment Variables

**Decision**:
- **CORS**: The FastAPI backend will use the `CORSMiddleware` to allow requests from the Docusaurus frontend. The allowed origins will be configured to match the Docusaurus development and production URLs.
- **Environment Variables**: We will use the `dotenv` package in the Docusaurus project to manage the `RAG_BACKEND_URL`. We will also use Docusaurus's `customFields` in `docusaurus.config.js` to make the environment variable available to the client-side code.

## T004: Docusaurus Build System Conflicts

**Decision**: No significant build system conflicts are anticipated.

**Rationale**:
- The chatbot is a standard React component.
- Docusaurus is built on React.
- As long as we follow standard React and Docusaurus practices, we should not encounter any major issues. Any minor CSS conflicts can be resolved with careful styling.