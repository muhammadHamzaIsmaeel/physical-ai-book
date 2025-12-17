# Railway.app Deployment Guide 🚂

## Overview
Deploy your RAG Chatbot backend to Railway.app - Fast, simple, and FREE!

**Why Railway?**
- ✅ Faster deployment than Render
- ✅ Better free tier (500 hours/month)
- ✅ Automatic GitHub integration
- ✅ Built-in database support
- ✅ No credit card required for free tier

---

## Step-by-Step Deployment (Urdu + English)

### Step 1: GitHub par Code Push Karo

Sabse pehle apna code GitHub par push karo:

```bash
# All files add karo
git add .

# Commit karo
git commit -m "Add Railway deployment configuration"

# GitHub par push karo
git push origin main
```

---

### Step 2: Railway Account Banao

1. **Railway.app par jao**: https://railway.app
2. **"Start a New Project"** click karo
3. **"Login With GitHub"** select karo
4. GitHub se sign in karo aur Railway ko access do

---

### Step 3: New Project Banao

#### Option A: GitHub Repo se Deploy (Recommended)

1. Railway dashboard mein **"New Project"** click karo
2. **"Deploy from GitHub repo"** select karo
3. Apna repository select karo: `muhammadhamzaismaeel/physical-ai-book`
4. Railway automatically detect karega aur deploy karega

#### Option B: CLI se Deploy (Advanced)

```bash
# Railway CLI install karo
npm i -g @railway/cli

# Login karo
railway login

# Project link karo
railway link

# Deploy karo
railway up
```

---

### Step 4: Service Configure Karo

1. **Root Directory Set Karo**:
   - Settings → "Root Directory" → `backend` likh kar save karo

2. **Build Settings**:
   - Railway automatically detect kar lega Python aur requirements.txt ko
   - Build Command: `pip install -r requirements.txt` (auto-detected)
   - Start Command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT` (auto-detected)

3. **Region Select Karo**:
   - Settings → Region → Closest region select karo (e.g., "us-west1")

---

### Step 5: Environment Variables Add Karo

**Bahut Important!** Yeh step skip mat karo!

1. Railway dashboard mein **"Variables"** tab par jao
2. Neeche diye gaye variables add karo:

```plaintext
GOOGLE_API_KEY=your_actual_gemini_api_key_here
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=ai_book_docs
DATABASE_URL=your_neon_postgres_connection_string
```

**Kahan se lao yeh values?**
- Apni local `backend/.env` file se copy karo
- Make sure exact same values ho

**How to Add:**
1. "New Variable" click karo
2. Variable name likho (e.g., `GOOGLE_API_KEY`)
3. Value paste karo
4. "Add" click karo
5. Repeat for all variables

---

### Step 6: Deploy Karo!

1. Sab variables add karne ke baad, Railway **automatically deploy** kar dega
2. Deployment logs dekhne ke liye **"Deployments"** tab par jao
3. Wait karo 2-5 minutes
4. Green checkmark dikhe to deployment successful! ✅

---

### Step 7: URL Copy Karo

1. **"Settings"** tab par jao
2. **"Domains"** section mein jao
3. **"Generate Domain"** click karo
4. Railway apko ek URL dega like:
   ```
   https://your-project-name.up.railway.app
   ```
5. Yeh URL copy karo - frontend mein use karenge!

---

### Step 8: Backend Test Karo

Terminal mein test karo:

```bash
# Health check
curl https://your-project-name.up.railway.app/

# Should return:
# {"message": "RAG Chatbot API is running."}

# Chat test
curl -X POST https://your-project-name.up.railway.app/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is URDF?", "history": []}'

# Should return streaming response
```

Agar response aa raha hai to backend working! ✅

---

## Frontend Update Karo

### Step 1: ChatWidget.js Update Karo

`src/components/ChatWidget/ChatWidget.js` file ko edit karo:

**Line 9 ko update karo:**

```javascript
const API_BASE_URL = process.env.NODE_ENV === 'production'
    ? 'https://YOUR-RAILWAY-URL.up.railway.app'  // ← Apna Railway URL yahan paste karo
    : 'http://localhost:8000';
```

**Example:**
```javascript
const API_BASE_URL = process.env.NODE_ENV === 'production'
    ? 'https://ai-book-backend-production.up.railway.app'
    : 'http://localhost:8000';
```

### Step 2: Git Push Karo

```bash
git add src/components/ChatWidget/ChatWidget.js
git commit -m "Update backend URL to Railway"
git push origin main
```

### Step 3: GitHub Pages Deploy Karo

```bash
# Build karo
npm run build

# Deploy karo
npm run deploy
```

Ya agar GitHub Actions use kar rahe ho to automatically deploy ho jayega!

---

## Testing on Live Site

1. **Apni site kholo**: https://muhammadhamzaismaeel.github.io/physical-ai-book/
2. **Chat bubble (💬)** click karo bottom-right corner mein
3. **Test question pucho**: "What is URDF?"
4. **Check karo**:
   - ✅ Chat opens
   - ✅ Message sends
   - ✅ Response streams in real-time
   - ✅ Markdown renders properly
   - ✅ Source links clickable hain

---

## Railway Free Tier Details

### What You Get (FREE):
- ✅ **500 execution hours/month** (better than Render's 750 hours)
- ✅ **No sleep/spin-down** (instant responses!)
- ✅ **$5 credit/month** (usually enough for hobby projects)
- ✅ **Automatic HTTPS**
- ✅ **GitHub auto-deployment**

### Limitations:
- After $5 credit exhausted, service stops
- Monitor usage in Railway dashboard
- Usually lasts full month for low-traffic apps

### How to Monitor Usage:
1. Railway dashboard → Your project
2. "Usage" tab → Check credit consumption
3. Typically uses ~$0.01-0.10 per day for chatbot

---

## Troubleshooting (Common Issues)

### 1. Deployment Failed ❌

**Problem**: Railway build fail ho raha hai

**Solution**:
- Logs check karo: Deployments tab → Latest deployment → View logs
- `requirements.txt` path check karo (should be in `backend/` folder)
- Root directory setting verify karo (should be `backend`)

### 2. Environment Variables Missing ⚠️

**Problem**: Backend start ho raha hai but errors de raha hai

**Solution**:
```bash
# Railway logs check karo
railway logs

# Common error: "GOOGLE_API_KEY not found"
# Fix: Variables tab mein jao aur missing variable add karo
```

### 3. CORS Error in Browser 🚫

**Problem**: Browser console mein CORS error

**Solution**:
- `backend/src/main.py` already updated hai with GitHub Pages URL
- Agar phir bhi error aa rahi hai, Railway URL ko bhi add karo:

```python
allow_origins=[
    "http://localhost:3000",
    "https://muhammadhamzaismaeel.github.io",
    "https://your-railway-url.up.railway.app",  # Add this
]
```

### 4. Slow First Response 🐌

**Problem**: Pehla request slow hai

**Solution**:
- Railway free tier mein **no sleep** hai, so yeh issue nahi hona chahiye
- Agar ho raha hai to cold start ho sakta hai (5-10 seconds)
- Subsequent requests fast honge

### 5. 503 Service Unavailable 💥

**Problem**: Backend down hai ya respond nahi kar raha

**Solution**:
```bash
# Railway dashboard mein jao
# Deployments → Latest deployment
# "Restart" button click karo
```

**Ya**:
```bash
railway up --detach
```

---

## Common Commands

### Using Railway CLI

```bash
# Install CLI
npm i -g @railway/cli

# Login
railway login

# Link project
railway link

# Deploy
railway up

# View logs (real-time)
railway logs

# Open dashboard
railway open

# Check status
railway status

# Environment variables
railway variables
```

---

## Comparison: Railway vs Render

| Feature | Railway | Render |
|---------|---------|--------|
| Free Tier | $5/month credit | 750 hours/month |
| Sleep/Spin-down | No ❌ | Yes (15 min) ⏱️ |
| First Request | Fast ⚡ | Slow (30-60s) 🐌 |
| Setup | Very Easy | Easy |
| Auto Deploy | Yes ✅ | Yes ✅ |
| Database | Built-in | External |

**Recommendation**: Railway is better for chatbot because no sleep = instant responses!

---

## Cost Optimization Tips

### 1. Monitor Usage
```bash
# Check credit usage
railway status

# View metrics
railway metrics
```

### 2. Optimize Code
- Use connection pooling for database
- Cache Qdrant results when possible
- Implement rate limiting

### 3. Set Usage Alerts
- Railway dashboard → Settings → Usage alerts
- Set alert at $4 (80% of free tier)

---

## Next Steps After Deployment

### 1. Custom Domain (Optional)
```bash
# Railway mein custom domain add kar sakte ho
# Settings → Domains → Add custom domain
```

### 2. Monitoring
- Railway dashboard mein built-in metrics
- Check logs regularly: `railway logs`

### 3. Auto-Deploy Setup
- Already enabled by default!
- Har GitHub push par automatic deploy

### 4. Database Backups
- Neon Postgres automatically backup karta hai
- No extra setup needed

---

## Quick Reference Card

### Railway Setup Checklist
- [ ] Railway account banaya (GitHub se)
- [ ] New project banaya
- [ ] GitHub repo connected
- [ ] Root directory = `backend`
- [ ] Environment variables add kiye (5 variables)
- [ ] Domain generated aur copy kiya
- [ ] Backend test kiya (curl command)
- [ ] Frontend updated (ChatWidget.js)
- [ ] GitHub Pages deploy kiya
- [ ] Live site test kiya

### Environment Variables Needed
```
✓ GOOGLE_API_KEY
✓ QDRANT_URL
✓ QDRANT_API_KEY
✓ QDRANT_COLLECTION_NAME
✓ DATABASE_URL
```

### Test Commands
```bash
# Health check
curl https://YOUR-URL.up.railway.app/

# Chat test
curl -X POST https://YOUR-URL.up.railway.app/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "test", "history": []}'
```

---

## Success Criteria ✅

Deployment successful hai agar:
1. ✅ Railway dashboard mein green checkmark
2. ✅ Domain URL accessible hai
3. ✅ Health endpoint (`/`) response de raha hai
4. ✅ Chat endpoint (`/chat`) streaming response de raha hai
5. ✅ Live site par chat widget kaam kar raha hai
6. ✅ Real-time responses aa rahe hain
7. ✅ Markdown render ho raha hai
8. ✅ Source links clickable hain

---

## Need Help?

### Check Logs
```bash
# Railway CLI se
railway logs

# Ya Railway dashboard mein
Deployments → Latest → View Logs
```

### Common Log Errors

**"Module not found"**
→ `requirements.txt` missing hai ya path wrong hai

**"Port already in use"**
→ Railway automatically assign karta hai, `$PORT` variable use karo

**"Database connection failed"**
→ `DATABASE_URL` check karo Variables tab mein

**"API key not found"**
→ Environment variables properly set karo

---

## Summary

**Railway deployment is:**
- ✅ Faster than Render
- ✅ No sleep issues
- ✅ Easier to use
- ✅ Better for real-time apps like chatbot
- ✅ Completely FREE for your use case

**Total FREE Stack:**
- Backend: Railway ($5 credit/month)
- Frontend: GitHub Pages (Free)
- Database: Neon Postgres (Free)
- Vector DB: Qdrant Cloud (Free)
- LLM: Gemini API (Free)

**Total Cost: ₹0/month** 🎉

---

## Deployment Complete! 🚀

Congratulations! Apka AI Book Chatbot ab live hai aur worldwide accessible hai!

**Your Live URLs:**
- Frontend: https://muhammadhamzaismaeel.github.io/physical-ai-book/
- Backend: https://your-project.up.railway.app/

Enjoy your fully functional RAG chatbot! 🤖📚
