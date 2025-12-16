# RAG Chatbot Deployment Guide

## Overview
This guide will help you deploy your AI Book chatbot:
- **Backend**: Deploy to Render.com (Free tier)
- **Frontend**: Already deployed on GitHub Pages ✅

## Prerequisites

Before deploying, make sure you have:
- [ ] GitHub account (you already have this)
- [ ] Render.com account (free - create at https://render.com)
- [ ] All environment variables ready:
  - `GOOGLE_API_KEY` (Gemini API key)
  - `QDRANT_URL` (Qdrant cloud URL)
  - `QDRANT_API_KEY` (Qdrant API key)
  - `QDRANT_COLLECTION_NAME` (your collection name)
  - `DATABASE_URL` (Neon Postgres connection string)

---

## Step 1: Deploy Backend to Render.com

### 1.1 Push Code to GitHub

First, commit and push your backend changes:

```bash
# Add all files
git add .

# Commit changes
git commit -m "Add backend deployment configuration for Render"

# Push to GitHub
git push origin main
```

### 1.2 Create Render Account

1. Go to https://render.com
2. Click "Get Started for Free"
3. Sign up with your GitHub account

### 1.3 Create New Web Service

1. In Render dashboard, click **"New +"** → **"Web Service"**
2. Connect your GitHub repository: `muhammadhamzaismaeel/physical-ai-book`
3. Configure the service:

   **Basic Settings:**
   - **Name**: `ai-book-chatbot-backend` (or any name you prefer)
   - **Region**: Choose closest to you (e.g., Oregon)
   - **Branch**: `main`
   - **Root Directory**: `backend`
   - **Runtime**: `Python 3`

   **Build & Deploy:**
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`

   **Instance Type:**
   - Select **"Free"** (this gives you 750 hours/month free)

### 1.4 Add Environment Variables

In the Render dashboard, scroll to **"Environment Variables"** section and add:

```
GOOGLE_API_KEY = your_gemini_api_key_here
QDRANT_URL = your_qdrant_cloud_url
QDRANT_API_KEY = your_qdrant_api_key
QDRANT_COLLECTION_NAME = your_collection_name
DATABASE_URL = your_neon_postgres_connection_string
PYTHON_VERSION = 3.11.0
```

**Important**: Make sure to copy the exact values from your local `backend/.env` file!

### 1.5 Deploy

1. Click **"Create Web Service"**
2. Wait 5-10 minutes for deployment to complete
3. You'll see logs in the dashboard
4. Once deployed, you'll get a URL like: `https://ai-book-chatbot-backend.onrender.com`

### 1.6 Test Backend

Test your deployed backend:

```bash
curl -X POST https://your-app-name.onrender.com/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is URDF?", "history": []}'
```

You should see streaming responses! ✅

---

## Step 2: Update Frontend Configuration

### 2.1 Update Backend URL in Frontend

Open `src/components/ChatWidget/ChatWidget.js` and update line 9 with your Render URL:

```javascript
const API_BASE_URL = process.env.NODE_ENV === 'production'
    ? 'https://YOUR-ACTUAL-RENDER-URL.onrender.com'  // Replace with your Render URL
    : 'http://localhost:8000';
```

**Example:**
```javascript
const API_BASE_URL = process.env.NODE_ENV === 'production'
    ? 'https://ai-book-chatbot-backend.onrender.com'
    : 'http://localhost:8000';
```

### 2.2 Commit and Push Frontend Changes

```bash
# Add the updated file
git add src/components/ChatWidget/ChatWidget.js

# Commit
git commit -m "Update backend URL to use deployed Render service"

# Push to GitHub
git push origin main
```

### 2.3 Deploy to GitHub Pages

Since your book is already deployed on GitHub Pages, you just need to rebuild:

```bash
# Build the site
npm run build

# Deploy to GitHub Pages (if using gh-pages package)
npm run deploy
```

**Or if you're using GitHub Actions:**
- The push to main will automatically trigger the deployment
- Wait 2-3 minutes for GitHub Actions to complete
- Check: https://muhammadhamzaismaeel.github.io/physical-ai-book/

---

## Step 3: Final Testing

### 3.1 Test on Live Site

1. Open your live site: https://muhammadhamzaismaeel.github.io/physical-ai-book/
2. Click the chat bubble (💬) in bottom-right corner
3. Ask a question: "What is URDF?"
4. Verify:
   - ✅ Chat opens
   - ✅ Message sends
   - ✅ Response streams in
   - ✅ Markdown renders properly
   - ✅ Source links are clickable

### 3.2 Check Browser Console

If chat doesn't work:
1. Open browser DevTools (F12)
2. Check Console tab for errors
3. Check Network tab to see if API calls are going through

---

## Troubleshooting

### Backend Issues

**Problem: Render deployment fails**
- Check logs in Render dashboard
- Verify `requirements.txt` is in `backend/` directory
- Ensure all environment variables are set

**Problem: Backend returns 500 errors**
- Check Render logs for Python errors
- Verify environment variables are correct
- Test database connection (Neon Postgres)
- Test Qdrant connection

**Problem: CORS errors in browser**
- Verify `backend/src/main.py` includes your GitHub Pages URL
- Check that URL matches exactly: `https://muhammadhamzaismaeel.github.io`

### Frontend Issues

**Problem: Chat widget shows but doesn't connect**
- Verify `API_BASE_URL` in `ChatWidget.js` matches your Render URL
- Check browser console for CORS errors
- Ensure backend is running (visit your Render URL in browser)

**Problem: Deployed site doesn't show changes**
- Clear browser cache (Ctrl+Shift+R)
- Wait for GitHub Pages deployment to complete
- Check GitHub Actions tab for deployment status

---

## Important Notes

### Render Free Tier Limitations

- ⏱️ **Spin Down**: Free tier services spin down after 15 minutes of inactivity
- 🕐 **First Request**: First request after spin-down takes 30-60 seconds to wake up
- 💾 **Hours**: 750 hours/month free (enough for development/demo)

**Solution**: Keep backend warm by pinging it every 10 minutes (optional):
- Use a service like UptimeRobot (free)
- Or add a cron job to ping your backend

### Cost Optimization

**Current Setup (FREE):**
- ✅ Backend: Render Free Tier
- ✅ Frontend: GitHub Pages (Free)
- ✅ Database: Neon Postgres Free Tier
- ✅ Vector DB: Qdrant Cloud Free Tier
- ✅ LLM: Gemini API Free Tier

**Total Monthly Cost: $0** 🎉

---

## Alternative Backend Deployment Options

If Render doesn't work for you, here are alternatives:

### 1. **Railway.app** (Free Tier)
```bash
# Install Railway CLI
npm i -g @railway/cli

# Login
railway login

# Deploy
railway up
```

### 2. **Fly.io** (Free Tier)
```bash
# Install Fly CLI
curl -L https://fly.io/install.sh | sh

# Deploy
fly launch
```

### 3. **Google Cloud Run** (Free Tier)
- More complex setup
- Requires Docker
- Best for production use

---

## Summary Checklist

### Backend Deployment
- [ ] Create Render account
- [ ] Create new web service from GitHub repo
- [ ] Configure build and start commands
- [ ] Add all environment variables
- [ ] Deploy and wait for completion
- [ ] Test backend with curl
- [ ] Copy your Render URL

### Frontend Deployment
- [ ] Update `ChatWidget.js` with Render URL
- [ ] Commit and push changes to GitHub
- [ ] Deploy to GitHub Pages (`npm run deploy`)
- [ ] Wait for deployment to complete
- [ ] Test on live site

### Final Verification
- [ ] Open live site
- [ ] Click chat bubble
- [ ] Send test message
- [ ] Verify streaming response
- [ ] Check markdown rendering
- [ ] Test source links
- [ ] Test on mobile device

---

## Getting Help

If you run into issues:

1. **Check Logs**:
   - Render: Dashboard → Logs tab
   - GitHub Pages: Repository → Actions tab
   - Browser: DevTools → Console

2. **Common Commands**:
   ```bash
   # Test backend locally
   cd backend
   uvicorn src.main:app --reload --port 8000

   # Test frontend locally
   npm start

   # Build frontend
   npm run build
   ```

3. **Environment Variables**:
   - Double-check all values in Render dashboard
   - Ensure no extra spaces or quotes
   - DATABASE_URL should start with `postgresql://`

---

## Success! 🎉

Once deployed, your AI Book chatbot will be:
- ✅ Accessible worldwide at your GitHub Pages URL
- ✅ Fully functional with streaming responses
- ✅ Professional UX with markdown rendering
- ✅ Running completely free on Render + GitHub Pages

Your users can now:
- Visit your book: https://muhammadhamzaismaeel.github.io/physical-ai-book/
- Click the chat bubble
- Ask questions about Physical AI and Humanoid Robotics
- Get instant, AI-powered answers with source citations

---

## Next Steps (Optional)

### 1. Custom Domain
- Add custom domain to GitHub Pages (e.g., `book.yourdomain.com`)
- Update CORS in backend to include custom domain

### 2. Analytics
- Add Google Analytics to track chat usage
- Monitor popular questions

### 3. Improvements
- Add rate limiting to prevent abuse
- Add user feedback buttons (👍 👎)
- Implement conversation history persistence
- Add authentication for admin features

### 4. Monitoring
- Set up UptimeRobot to monitor backend
- Add error tracking (Sentry)
- Monitor API usage and costs

---

**Need help?** Check the logs in Render dashboard or browser console for specific error messages.
