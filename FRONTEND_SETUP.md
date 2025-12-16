# Frontend Setup Guide - RAG Chatbot Integration

## ✅ Setup Complete!

Your ChatWidget is already integrated with Docusaurus! All the necessary files are in place:

- ✅ `src/components/ChatWidget/ChatWidget.js` - React component
- ✅ `src/components/ChatWidget/ChatWidget.css` - Styling
- ✅ `src/theme/Root.js` - Docusaurus theme integration

## 🚀 Running the Frontend

### Prerequisites

1. **Backend must be running** on `http://localhost:8000`
   ```bash
   cd backend
   uvicorn src.main:app --reload --port 8000
   ```

2. **Data must be ingested** (run once)
   ```bash
   python scripts/ingest.py --path ./
   ```

### Start Docusaurus

```bash
# Install dependencies (if not already done)
npm install

# Start the development server
npm start
```

The site will open at `http://localhost:3000`

## 🎯 Testing the ChatWidget

Once the Docusaurus site loads:

1. **Look for the Chat Bubble** in the bottom-right corner (blue circle with "?")
2. **Click the bubble** to open the chat window
3. **Type a question** like:
   - "What is URDF?"
   - "How do I set up Isaac Sim?"
   - "Explain ROS 2 nodes"
4. **Press Enter or click Send**
5. **Watch the streaming response** appear in real-time
6. **Check the sources** at the end of the response

## 📱 Features

### Chat Widget Features:
- ✅ Fixed position (bottom-right corner)
- ✅ Collapsible window
- ✅ Real-time streaming responses
- ✅ Message history (maintains last 4 messages for context)
- ✅ Loading indicators
- ✅ Error handling
- ✅ Dark/light mode support (uses Docusaurus CSS variables)
- ✅ Mobile responsive

### ChatWidget Controls:
- **Chat Bubble**: Click to open/close
- **Close Button (X)**: Close the chat window
- **Input Field**: Type your question
- **Send Button**: Submit the question
- **Enter Key**: Also submits the question

## 🎨 Customization

### Change Colors
Edit `src/components/ChatWidget/ChatWidget.css`:
```css
/* Primary color (bubble, user messages, send button) */
--ifm-color-primary: #2e8555;

/* Background colors */
--ifm-background-color: white;

/* Text colors */
--ifm-font-color-base: #000;
```

### Change Position
Edit `ChatWidget.css`:
```css
.chat-widget-container {
    bottom: 20px;  /* Distance from bottom */
    right: 20px;   /* Distance from right */
}
```

### Change Size
Edit `ChatWidget.css`:
```css
.chat-window {
    width: 350px;   /* Window width */
    height: 500px;  /* Window height */
}
```

## 🔧 Backend Configuration

The ChatWidget connects to `http://localhost:8000/chat` by default.

To change the backend URL, edit `src/components/ChatWidget/ChatWidget.js`:
```javascript
const response = await fetch('http://localhost:8000/chat', {
    // Change URL here for production
```

For production, you'll want to use an environment variable:
```javascript
const BACKEND_URL = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';

const response = await fetch(`${BACKEND_URL}/chat`, {
```

## 🐛 Troubleshooting

### Chat bubble not showing
- **Check**: Is Docusaurus running? (`npm start`)
- **Check**: Open browser console for errors
- **Solution**: Clear browser cache and refresh

### "Couldn't connect to AI assistant" error
- **Check**: Is backend running? (`uvicorn src.main:app --reload --port 8000`)
- **Check**: CORS configured in `backend/src/main.py`
- **Check**: Backend logs for errors
- **Solution**: Restart backend server

### No response to questions
- **Check**: Is data ingested? Run `python scripts/ingest.py --path ./`
- **Check**: Qdrant collection has data
- **Check**: Postgres has records
- **Check**: Backend logs for errors

### Streaming not working
- **Check**: Browser supports Server-Sent Events (all modern browsers do)
- **Check**: Backend returns `text/event-stream` content type
- **Check**: No proxy/firewall blocking streaming

## 📊 What's Happening Behind the Scenes

1. **User types question** → Frontend captures input
2. **Frontend sends request** → `POST http://localhost:8000/chat`
3. **Backend generates embedding** → Gemini API
4. **Backend searches Qdrant** → Find similar chunks
5. **Backend fetches metadata** → Postgres database
6. **Backend generates answer** → Gemini with context
7. **Backend streams response** → Server-Sent Events
8. **Frontend displays chunks** → Real-time updates
9. **Response complete** → `[DONE]` signal received

## ✨ Next Steps

### For Development:
1. Test with different questions
2. Try multiple conversation turns
3. Test error scenarios (backend down, network issues)
4. Test on different browsers
5. Test on mobile devices

### For Production:
1. Deploy backend to cloud (Render, Vercel, Railway)
2. Update frontend URL to production backend
3. Build Docusaurus: `npm run build`
4. Deploy to GitHub Pages: `npm run deploy`
5. Update CORS to allow production domain

## 🎉 Success Criteria

Your frontend is working correctly if:
- ✅ Chat bubble visible on all pages
- ✅ Window opens/closes smoothly
- ✅ Questions can be submitted
- ✅ Responses stream in real-time
- ✅ Sources are displayed
- ✅ Conversation history maintained
- ✅ Error messages show when backend unavailable
- ✅ UI responsive on mobile

## 📝 Testing Checklist

- [ ] Backend running on port 8000
- [ ] Data ingested successfully
- [ ] Docusaurus running on port 3000
- [ ] Chat bubble visible
- [ ] Can open/close chat window
- [ ] Can send messages
- [ ] Receives streaming responses
- [ ] Sources displayed correctly
- [ ] Multiple messages work
- [ ] Error handling works
- [ ] Works in different browsers
- [ ] Works on mobile

## 🚀 Quick Start Commands

```bash
# Terminal 1: Start Backend
cd backend
uvicorn src.main:app --reload --port 8000

# Terminal 2: Start Frontend
npm start

# Open browser to http://localhost:3000
# Click chat bubble in bottom-right
# Ask: "What is URDF?"
```

**Congratulations! Your RAG Chatbot is fully integrated!** 🎊
