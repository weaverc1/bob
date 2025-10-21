#!/bin/bash
# AI Mower Crew - Quick Run Script

echo "================================================"
echo "AI MOWER CREW - Starting..."
echo "================================================"
echo ""

# Check if virtual environment exists
if [ ! -d "$HOME/crewai-env" ]; then
    echo "❌ Error: Virtual environment not found at ~/crewai-env"
    echo "Please create it first with:"
    echo "  python3.12 -m venv ~/crewai-env"
    echo "  source ~/crewai-env/bin/activate"
    echo "  pip install crewai crewai-tools"
    exit 1
fi

# Activate virtual environment
echo "Activating virtual environment..."
source "$HOME/crewai-env/bin/activate"

# Check if CrewAI is installed
if ! python3 -c "import crewai" 2>/dev/null; then
    echo "❌ Error: CrewAI not installed in virtual environment"
    echo "Install with: pip install crewai crewai-tools"
    exit 1
fi

echo "✅ Virtual environment activated"
echo "✅ CrewAI installed"
echo ""

# Navigate to project directory
cd "$(dirname "$0")"

echo "Running AI Mower Crew..."
echo "================================================"
echo ""

# Run the crew
python3 main.py "$@"

echo ""
echo "================================================"
echo "Crew execution complete!"
echo "================================================"
