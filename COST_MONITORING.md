# Cost Monitoring Guide for AI Mower Crew

## Current Configuration

### Model: GPT-4o-mini
- **Cost:** ~$0.15 per 1M input tokens, ~$0.60 per 1M output tokens
- **Comparison:** ~15x cheaper than GPT-4
- **Quality:** Excellent for technical tasks like architecture design

### Token Limits
- **Max tokens per response:** 4,000 (configured in .env)
- **This prevents:** Runaway costs from overly long responses

## Estimated Costs for This Project

### Per Task (Approximate)
- Simple task (package search): ~$0.02-0.05
- Medium task (architecture design): ~$0.10-0.20
- Complex task (full analysis): ~$0.20-0.40

### Full Crew Run (12 Tasks)
- **Estimated total:** $1.50-$3.00
- **Breakdown:**
  - Architecture design: ~$0.30
  - Safety analysis: ~$0.25
  - Navigation stack: ~$0.20
  - Differential drive: ~$0.20
  - Package search: ~$0.15
  - URDF design: ~$0.15
  - Gazebo worlds: ~$0.15
  - Testing protocols: ~$0.20
  - Feasibility review: ~$0.15
  - Launch system: ~$0.15
  - Controller integration: ~$0.20
  - Knowledge base: ~$0.20

## How to Monitor Costs

### 1. OpenAI Dashboard
Visit: https://platform.openai.com/usage

**Features:**
- Real-time usage tracking
- Cost breakdown by day/model
- Set usage limits and alerts

### 2. Set Usage Limits (Recommended)
1. Go to: https://platform.openai.com/account/limits
2. Set a **hard limit** (e.g., $10/month)
3. Set a **soft limit** alert (e.g., $5/month)

### 3. Monitor During Crew Execution
The crew will show:
- Number of LLM calls
- Tokens used per task
- Time per agent

### 4. Post-Run Analysis
Check `output/crew_results.md` for:
- Total execution time
- Number of agent interactions
- Complexity of outputs

## Cost-Saving Tips

### ✅ Already Implemented
- Using GPT-4o-mini (15x cheaper than GPT-4)
- Token limits (4,000 max per response)
- Environment variables for easy model switching

### 🔧 Additional Savings
1. **Run Specific Tasks Only**
   - Edit `main.py` to run only needed tasks
   - Save full crew runs for major milestones

2. **Use Caching**
   - Results are saved in `output/`
   - Review before re-running

3. **Iterative Approach**
   - Run architecture first
   - Review and refine
   - Then run remaining tasks

4. **Switch to Local LLM for Experimentation**
   - Use Ollama with Llama 3 (free)
   - Switch back to GPT-4o-mini for final production runs

## Budget Planning

### Conservative Budget
- **Initial architecture run:** $3
- **Refinement runs (2-3x):** $6-9
- **Total for project setup:** ~$10-15

### Generous Budget
- **Multiple iterations:** $20-30
- **Includes experimentation and refinement**

## Emergency Stop

If costs start spiraling:

1. **Stop the crew:** Press Ctrl+C
2. **Check OpenAI dashboard:** https://platform.openai.com/usage
3. **Set hard limit:** https://platform.openai.com/account/limits
4. **Switch to local LLM:** See QUICKSTART.md for Ollama setup

## Cost Comparison

| Approach | Cost | Quality | Speed |
|----------|------|---------|-------|
| GPT-4 | $30-60/run | Excellent | Fast |
| **GPT-4o-mini** | **$1.50-3/run** | **Excellent** | **Fast** |
| GPT-3.5-turbo | $0.50-1/run | Good | Fast |
| Ollama (Local) | FREE | Good-Very Good | Slow |

## Monitoring Commands

### Check total API usage
```bash
# View OpenAI dashboard
xdg-open https://platform.openai.com/usage  # Linux
# or visit in browser
```

### View crew output
```bash
cat ~/ai_mower_crew/output/crew_results.md
```

### Check configuration
```bash
cat ~/ai_mower_crew/.env
```

---

## Current Status

✅ **Configured:** GPT-4o-mini with 4K token limit
✅ **API Key:** Secured in .env file (not committed to git)
✅ **Estimated cost per run:** $1.50-$3.00

**Ready to run cost-effectively!**

For questions or to adjust settings, edit `.env` file in project root.
