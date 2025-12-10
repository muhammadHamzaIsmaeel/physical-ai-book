---
id: 024
title: Corrected Docusaurus Sidebar Document IDs
stage: refactor
date: 2025-12-10
surface: agent
model: unknown
feature: 001-physical-ai-book
branch: main
user: unknown
command: yar ek ya error arha hai "C:\Users\ADVANCES--PC\Desktop\ai_book>npm start ... Invalid sidebar file at "sidebars.js". These sidebar document ids do not exist: - appendices/lab-build-guides/index - capstone-butler/index - sim-to-real/index ...
labels: ["bug-fix", "docusaurus", "sidebar", "configuration"]
links:
  spec: specs/001-physical-ai-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - sidebars.js
tests: []
---

## Prompt

yar ek ya error arha hai "C:\Users\ADVANCES--PC\Desktop\ai_book>npm start     

> physical-ai-humanoid-robotics-book@1.0.0 start
> docusaurus start

[INFO] Starting the development server...
[WARNING] The `siteConfig.onBrokenMarkdownLinks` config option is deprecated and will be removed in Docusaurus v4.
Please migrate and move this option to `siteConfig.markdown.hooks.onBrokenMarkdownLinks` instead.
[ERROR] Loading of version failed for version current

[ERROR] Error: Invalid sidebar file at "sidebars.js".
These sidebar document ids do not exist:
- appendices/lab-build-guides/index
- capstone-butler/index
- sim-to-real/index

Available document ids are:
- appendices/future-roadmap
- appendices/future-roadmap/future-roadmap
- appendices/lab-build-guides
- appendices/lab-build-guides/economy-tier
- appendices/lab-build-guides/lab-build-guides
- appendices/lab-build-guides/mid-tier
- appendices/lab-build-guides/premium-tier
- appendices/troubleshooting-bible
- bipedal-locomotion/index
- bipedal-locomotion/quiz
- capstone-butler/capstone-butler
- dexterous-manipulation/index
- dexterous-manipulation/quiz
- hardware-2026/hardware-2026-quiz
- hardware-2026/index
- intro
- isaac-platform/diagrams
- isaac-platform/index
- isaac-platform/quiz
- perception-stack/perception-stack
- perception-stack/quiz
- ros2-fundamentals/debugging-tools
- ros2-fundamentals/index
- ros2-fundamentals/nodes-topics
- ros2-fundamentals/parameters-launch
- ros2-fundamentals/quiz
- ros2-fundamentals/services-actions
- sim-to-real/sim-to-real
- simulation-ecosystem/index
- simulation-ecosystem/quiz
- urdf-digital-twins/index
- urdf-digital-twins/quiz
- urdf-digital-twins/urdf-basics
- vla-models/index
- vla-models/quiz
- voice-to-action/index
- voice-to-action/quiz
- why-physical-ai/index
- why-physical-ai/why-physical-ai-quiz

    at Object.checkSidebarsDocIds (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\sidebars\utils.js:245:19)
    at doLoadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:102:19)
    at async loadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:119:16)
    at async Promise.all (index 0)
    at async Object.loadContent (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\index.js:152:33)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:40:23       
    at async Promise.all (index 0)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:146:25      
    at async loadSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\site.js:155:45)      
    at async createReloadableSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\commands\start\utils.js:62:16)
[INFO] Docusaurus version: 3.9.2
Node version: v22.16.0

C:\Users\ADVANCES--PC\Desktop\ai_book>""

## Prompt


yar ek ya error arha hai "C:\Users\ADVANCES--PC\Desktop\ai_book>npm start     

> physical-ai-humanoid-robotics-book@1.0.0 start
> docusaurus start

[INFO] Starting the development server...
[WARNING] The `siteConfig.onBrokenMarkdownLinks` config option is deprecated and will be removed in Docusaurus v4.
Please migrate and move this option to `siteConfig.markdown.hooks.onBrokenMarkdownLinks` instead.
[ERROR] Loading of version failed for version current

[ERROR] Error: Invalid sidebar file at "sidebars.js".
These sidebar document ids do not exist:
- appendices/lab-build-guides/index
- capstone-butler/index
- sim-to-real/index

Available document ids are:
- appendices/future-roadmap
- appendices/future-roadmap/future-roadmap
- appendices/lab-build-guides
- appendices/lab-build-guides/economy-tier
- appendices/lab-build-guides/lab-build-guides
- appendices/lab-build-guides/mid-tier
- appendices/lab-build-guides/premium-tier
- appendices/troubleshooting-bible
- bipedal-locomotion/index
- bipedal-locomotion/quiz
- capstone-butler/capstone-butler
- dexterous-manipulation/index
- dexterous-manipulation/quiz
- hardware-2026/hardware-2026-quiz
- hardware-2026/index
- intro
- isaac-platform/diagrams
- isaac-platform/index
- isaac-platform/quiz
- perception-stack/perception-stack
- perception-stack/quiz
- ros2-fundamentals/debugging-tools
- ros2-fundamentals/index
- ros2-fundamentals/nodes-topics
- ros2-fundamentals/parameters-launch
- ros2-fundamentals/quiz
- ros2-fundamentals/services-actions
- sim-to-real/sim-to-real
- simulation-ecosystem/index
- simulation-ecosystem/quiz
- urdf-digital-twins/index
- urdf-digital-twins/quiz
- urdf-digital-twins/urdf-basics
- vla-models/index
- vla-models/quiz
- voice-to-action/index
- voice-to-action/quiz
- why-physical-ai/index
- why-physical-ai/why-physical-ai-quiz

    at Object.checkSidebarsDocIds (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\sidebars\utils.js:245:19)
    at doLoadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:102:19)
    at async loadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:119:16)
    at async Promise.all (index 0)
    at async Object.loadContent (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\index.js:152:33)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:40:23       
    at async Promise.all (index 0)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:146:25      
    at async loadSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\site.js:155:45)      
    at async createReloadableSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\commands\start\utils.js:62:16)
[INFO] Docusaurus version: 3.9.2
Node version: v22.16.0

C:\Users\ADVANCES--PC\Desktop\ai_book>""

## Prompt


yar ek ya error arha hai "C:\Users\ADVANCES--PC\Desktop\ai_book>npm start     

> physical-ai-humanoid-robotics-book@1.0.0 start
> docusaurus start

[INFO] Starting the development server...
[WARNING] The `siteConfig.onBrokenMarkdownLinks` config option is deprecated and will be removed in Docusaurus v4.
Please migrate and move this option to `siteConfig.markdown.hooks.onBrokenMarkdownLinks` instead.
[ERROR] Loading of version failed for version current

[ERROR] Error: Invalid sidebar file at "sidebars.js".
These sidebar document ids do not exist:
- appendices/lab-build-guides/index
- capstone-butler/index
- sim-to-real/index

Available document ids are:
- appendices/future-roadmap
- appendices/future-roadmap/future-roadmap
- appendices/lab-build-guides
- appendices/lab-build-guides/economy-tier
- appendices/lab-build-guides/lab-build-guides
- appendices/lab-build-guides/mid-tier
- appendices/lab-build-guides/premium-tier
- appendices/troubleshooting-bible
- bipedal-locomotion/index
- bipedal-locomotion/quiz
- capstone-butler/capstone-butler
- dexterous-manipulation/index
- dexterous-manipulation/quiz
- hardware-2026/hardware-2026-quiz
- hardware-2026/index
- intro
- isaac-platform/diagrams
- isaac-platform/index
- isaac-platform/quiz
- perception-stack/perception-stack
- perception-stack/quiz
- ros2-fundamentals/debugging-tools
- ros2-fundamentals/index
- ros2-fundamentals/nodes-topics
- ros2-fundamentals/parameters-launch
- ros2-fundamentals/quiz
- ros2-fundamentals/services-actions
- sim-to-real/sim-to-real
- simulation-ecosystem/index
- simulation-ecosystem/quiz
- urdf-digital-twins/index
- urdf-digital-twins/quiz
- urdf-digital-twins/urdf-basics
- vla-models/index
- vla-models/quiz
- voice-to-action/index
- voice-to-action/quiz
- why-physical-ai/index
- why-physical-ai/why-physical-ai-quiz

    at Object.checkSidebarsDocIds (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\sidebars\utils.js:245:19)
    at doLoadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:102:19)
    at async loadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:119:16)
    at async Promise.all (index 0)
    at async Object.loadContent (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\index.js:152:33)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:40:23       
    at async Promise.all (index 0)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:146:25      
    at async loadSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\site.js:155:45)      
    at async createReloadableSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\commands\start\utils.js:62:16)
[INFO] Docusaurus version: 3.9.2
Node version: v22.16.0

C:\Users\ADVANCES--PC\Desktop\ai_book>""

## Prompt


yar ek ya error arha hai "C:\Users\ADVANCES--PC\Desktop\ai_book>npm start     

> physical-ai-humanoid-robotics-book@1.0.0 start
> docusaurus start

[INFO] Starting the development server...
[WARNING] The `siteConfig.onBrokenMarkdownLinks` config option is deprecated and will be removed in Docusaurus v4.
Please migrate and move this option to `siteConfig.markdown.hooks.onBrokenMarkdownLinks` instead.
[ERROR] Loading of version failed for version current

[ERROR] Error: Invalid sidebar file at "sidebars.js".
These sidebar document ids do not exist:
- appendices/lab-build-guides/index
- capstone-butler/index
- sim-to-real/index

Available document ids are:
- appendices/future-roadmap
- appendices/future-roadmap/future-roadmap
- appendices/lab-build-guides
- appendices/lab-build-guides/economy-tier
- appendices/lab-build-guides/lab-build-guides
- appendices/lab-build-guides/mid-tier
- appendices/lab-build-guides/premium-tier
- appendices/troubleshooting-bible
- bipedal-locomotion/index
- bipedal-locomotion/quiz
- capstone-butler/capstone-butler
- dexterous-manipulation/index
- dexterous-manipulation/quiz
- hardware-2026/hardware-2026-quiz
- hardware-2026/index
- intro
- isaac-platform/diagrams
- isaac-platform/index
- isaac-platform/quiz
- perception-stack/perception-stack
- perception-stack/quiz
- ros2-fundamentals/debugging-tools
- ros2-fundamentals/index
- ros2-fundamentals/nodes-topics
- ros2-fundamentals/parameters-launch
- ros2-fundamentals/quiz
- ros2-fundamentals/services-actions
- sim-to-real/sim-to-real
- simulation-ecosystem/index
- simulation-ecosystem/quiz
- urdf-digital-twins/index
- urdf-digital-twins/quiz
- urdf-digital-twins/urdf-basics
- vla-models/index
- vla-models/quiz
- voice-to-action/index
- voice-to-action/quiz
- why-physical-ai/index
- why-physical-ai/why-physical-ai-quiz

    at Object.checkSidebarsDocIds (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\sidebars\utils.js:245:19)
    at doLoadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:102:19)
    at async loadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:119:16)
    at async Promise.all (index 0)
    at async Object.loadContent (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\index.js:152:33)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:40:23       
    at async Promise.all (index 0)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:146:25      
    at async loadSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\site.js:155:45)      
    at async createReloadableSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\commands\start\utils.js:62:16)
[INFO] Docusaurus version: 3.9.2
Node version: v22.16.0

C:\Users\ADVANCES--PC\Desktop\ai_book>""

## Prompt


yar ek ya error arha hai "C:\Users\ADVANCES--PC\Desktop\ai_book>npm start     

> physical-ai-humanoid-robotics-book@1.0.0 start
> docusaurus start

[INFO] Starting the development server...
[WARNING] The `siteConfig.onBrokenMarkdownLinks` config option is deprecated and will be removed in Docusaurus v4.
Please migrate and move this option to `siteConfig.markdown.hooks.onBrokenMarkdownLinks` instead.
[ERROR] Loading of version failed for version current

[ERROR] Error: Invalid sidebar file at "sidebars.js".
These sidebar document ids do not exist:
- appendices/lab-build-guides/index
- capstone-butler/index
- sim-to-real/index

Available document ids are:
- appendices/future-roadmap
- appendices/future-roadmap/future-roadmap
- appendices/lab-build-guides
- appendices/lab-build-guides/economy-tier
- appendices/lab-build-guides/lab-build-guides
- appendices/lab-build-guides/mid-tier
- appendices/lab-build-guides/premium-tier
- appendices/troubleshooting-bible
- bipedal-locomotion/index
- bipedal-locomotion/quiz
- capstone-butler/capstone-butler
- dexterous-manipulation/index
- dexterous-manipulation/quiz
- hardware-2026/hardware-2026-quiz
- hardware-2026/index
- intro
- isaac-platform/diagrams
- isaac-platform/index
- isaac-platform/quiz
- perception-stack/perception-stack
- perception-stack/quiz
- ros2-fundamentals/debugging-tools
- ros2-fundamentals/index
- ros2-fundamentals/nodes-topics
- ros2-fundamentals/parameters-launch
- ros2-fundamentals/quiz
- ros2-fundamentals/services-actions
- sim-to-real/sim-to-real
- simulation-ecosystem/index
- simulation-ecosystem/quiz
- urdf-digital-twins/index
- urdf-digital-twins/quiz
- urdf-digital-twins/urdf-basics
- vla-models/index
- vla-models/quiz
- voice-to-action/index
- voice-to-action/quiz
- why-physical-ai/index
- why-physical-ai/why-physical-ai-quiz

    at Object.checkSidebarsDocIds (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\sidebars\utils.js:245:19)
    at doLoadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:102:19)
    at async loadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:119:16)
    at async Promise.all (index 0)
    at async Object.loadContent (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\index.js:152:33)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:40:23       
    at async Promise.all (index 0)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:146:25      
    at async loadSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\site.js:155:45)      
    at async createReloadableSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\commands\start\utils.js:62:16)
[INFO] Docusaurus version: 3.9.2
Node version: v22.16.0

C:\Users\ADVANCES--PC\Desktop\ai_book>""

## Prompt


yar ek ya error arha hai "C:\Users\ADVANCES--PC\Desktop\ai_book>npm start     

> physical-ai-humanoid-robotics-book@1.0.0 start
> docusaurus start

[INFO] Starting the development server...
[WARNING] The `siteConfig.onBrokenMarkdownLinks` config option is deprecated and will be removed in Docusaurus v4.
Please migrate and move this option to `siteConfig.markdown.hooks.onBrokenMarkdownLinks` instead.
[ERROR] Loading of version failed for version current

[ERROR] Error: Invalid sidebar file at "sidebars.js".
These sidebar document ids do not exist:
- appendices/lab-build-guides/index
- capstone-butler/index
- sim-to-real/index

Available document ids are:
- appendices/future-roadmap
- appendices/future-roadmap/future-roadmap
- appendices/lab-build-guides
- appendices/lab-build-guides/economy-tier
- appendices/lab-build-guides/lab-build-guides
- appendices/lab-build-guides/mid-tier
- appendices/lab-build-guides/premium-tier
- appendices/troubleshooting-bible
- bipedal-locomotion/index
- bipedal-locomotion/quiz
- capstone-butler/capstone-butler
- dexterous-manipulation/index
- dexterous-manipulation/quiz
- hardware-2026/hardware-2026-quiz
- hardware-2026/index
- intro
- isaac-platform/diagrams
- isaac-platform/index
- isaac-platform/quiz
- perception-stack/perception-stack
- perception-stack/quiz
- ros2-fundamentals/debugging-tools
- ros2-fundamentals/index
- ros2-fundamentals/nodes-topics
- ros2-fundamentals/parameters-launch
- ros2-fundamentals/quiz
- ros2-fundamentals/services-actions
- sim-to-real/sim-to-real
- simulation-ecosystem/index
- simulation-ecosystem/quiz
- urdf-digital-twins/index
- urdf-digital-twins/quiz
- urdf-digital-twins/urdf-basics
- vla-models/index
- vla-models/quiz
- voice-to-action/index
- voice-to-action/quiz
- why-physical-ai/index
- why-physical-ai/why-physical-ai-quiz

    at Object.checkSidebarsDocIds (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\sidebars\utils.js:245:19)
    at doLoadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:102:19)
    at async loadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:119:16)
    at async Promise.all (index 0)
    at async Object.loadContent (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\index.js:152:33)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:40:23       
    at async Promise.all (index 0)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:146:25      
    at async loadSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\site.js:155:45)      
    at async createReloadableSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\commands\start\utils.js:62:16)
[INFO] Docusaurus version: 3.9.2
Node version: v22.16.0

C:\Users\ADVANCES--PC\Desktop\ai_book>""

## Prompt


yar ek ya error arha hai "C:\Users\ADVANCES--PC\Desktop\ai_book>npm start     

> physical-ai-humanoid-robotics-book@1.0.0 start
> docusaurus start

[INFO] Starting the development server...
[WARNING] The `siteConfig.onBrokenMarkdownLinks` config option is deprecated and will be removed in Docusaurus v4.
Please migrate and move this option to `siteConfig.markdown.hooks.onBrokenMarkdownLinks` instead.
[ERROR] Loading of version failed for version current

[ERROR] Error: Invalid sidebar file at "sidebars.js".
These sidebar document ids do not exist:
- appendices/lab-build-guides/index
- capstone-butler/index
- sim-to-real/index

Available document ids are:
- appendices/future-roadmap
- appendices/future-roadmap/future-roadmap
- appendices/lab-build-guides
- appendices/lab-build-guides/economy-tier
- appendices/lab-build-guides/lab-build-guides
- appendices/lab-build-guides/mid-tier
- appendices/lab-build-guides/premium-tier
- appendices/troubleshooting-bible
- bipedal-locomotion/index
- bipedal-locomotion/quiz
- capstone-butler/capstone-butler
- dexterous-manipulation/index
- dexterous-manipulation/quiz
- hardware-2026/hardware-2026-quiz
- hardware-2026/index
- intro
- isaac-platform/diagrams
- isaac-platform/index
- isaac-platform/quiz
- perception-stack/perception-stack
- perception-stack/quiz
- ros2-fundamentals/debugging-tools
- ros2-fundamentals/index
- ros2-fundamentals/nodes-topics
- ros2-fundamentals/parameters-launch
- ros2-fundamentals/quiz
- ros2-fundamentals/services-actions
- sim-to-real/sim-to-real
- simulation-ecosystem/index
- simulation-ecosystem/quiz
- urdf-digital-twins/index
- urdf-digital-twins/quiz
- urdf-digital-twins/urdf-basics
- vla-models/index
- vla-models/quiz
- voice-to-action/index
- voice-to-action/quiz
- why-physical-ai/index
- why-physical-ai/why-physical-ai-quiz

    at Object.checkSidebarsDocIds (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\sidebars\utils.js:245:19)
    at doLoadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:102:19)
    at async loadVersion (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:119:16)
    at async Promise.all (index 0)
    at async Object.loadContent (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\plugin-content-docs\lib\index.js:152:33)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:40:23       
    at async Promise.all (index 0)
    at async C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:146:25      
    at async loadSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\server\site.js:155:45)      
    at async createReloadableSite (C:\Users\ADVANCES--PC\Desktop\ai_book\node_modules\@docusaurus\core\lib\commands\start\utils.js:62:16)
[INFO] Docusaurus version: 3.9.2
Node version: v22.16.0

C:\Users\ADVANCES--PC\Desktop\ai_book>""