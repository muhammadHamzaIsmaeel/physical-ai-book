I've applied the fixes to `sidebars.js` and `docusaurus.config.js`.

The specific changes were:
  - In `sidebars.js`, I updated the document IDs for the "Quiz" sections of "Why Physical AI Is the Next Frontier" and "The Hardware You Actually Need in 2026" to `why-physical-ai/why-physical-ai-quiz` and `hardware-2026/hardware-2026-quiz` respectively.
  - In `docusaurus.config.js`, I moved `onBrokenMarkdownLinks: 'warn'` from the top-level configuration to `markdown.hooks.onBrokenMarkdownLinks` to address the deprecation warning.

Due to a technical limitation, I cannot execute `npm start` directly from here to verify the fix. Please run `npm start` in your terminal to confirm that the issues are resolved.