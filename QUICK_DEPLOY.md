# Quick Deploy Checklist ⚡

## Backend Deployment (5 minutes)

### 1. Push to GitHub
```bash
git add .
git commit -m "Add deployment configuration"
git push origin main
```

### 2. Deploy to Render.com
1. Go to https://render.com → Sign up with GitHub
2. Click **New +** → **Web Service**
3. Select repo: `muhammadhamzaismaeel/physical-ai-book`
4. Settings:
   - Name: `ai-book-chatbot-backend`
   - Root Directory: `backend`
   - Build: `pip install -r requirements.txt`
   - Start: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
   - Instance: **Free**

5. Add Environment Variables:
   ```
   GOOGLE_API_KEY = (your Gemini key)
   QDRANT_URL = (your Qdrant URL)
   QDRANT_API_KEY = (your Qdrant key)
   QDRANT_COLLECTION_NAME = (your collection)
   DATABASE_URL = (your Neon Postgres URL)
   PYTHON_VERSION = 3.11.0
   ```

6. Click **Create Web Service**
7. Wait 5-10 minutes
8. Copy your Render URL (e.g., `https://ai-book-chatbot-backend.onrender.com`)

### 3. Test Backend
```bash
curl https://YOUR-RENDER-URL.onrender.com/
# Should return: {"message": "RAG Chatbot API is running."}
```

---

## Frontend Update (2 minutes)

### 1. Update Backend URL
Edit `src/components/ChatWidget/ChatWidget.js` line 9:
```javascript
const API_BASE_URL = process.env.NODE_ENV === 'production'
    ? 'https://YOUR-ACTUAL-RENDER-URL.onrender.com'  // ← Update this
    : 'http://localhost:8000';
```

### 2. Deploy to GitHub Pages
```bash
git add src/components/ChatWidget/ChatWidget.js
git commit -m "Update backend URL for production"
git push origin main

# Build and deploy
npm run build
npm run deploy  # or let GitHub Actions handle it
```

### 3. Test Live Site
1. Open: https://muhammadhamzaismaeel.github.io/physical-ai-book/
2. Click chat bubble (💬)
3. Ask: "What is URDF?"
4. Verify streaming response works ✅

---

## Troubleshooting

### Backend not responding?
- Check Render dashboard logs
- Verify environment variables are set
- First request after 15min takes 30-60s (free tier spin-up)

### CORS errors?
- Check `backend/src/main.py` includes GitHub Pages URL
- Verify URL format: `https://muhammadhamzaismaeel.github.io`

### Chat not connecting?
- Check browser console (F12)
- Verify Render URL in `ChatWidget.js` is correct
- Clear browser cache (Ctrl+Shift+R)

---

## That's it! 🎉

Your chatbot is now live and accessible worldwide for **FREE**!

**Cost Breakdown:**
- Backend (Render): $0/month
- Frontend (GitHub Pages): $0/month
- Database (Neon): $0/month
- Vector DB (Qdrant): $0/month
- LLM (Gemini): $0/month
- **Total: $0/month** 💰
