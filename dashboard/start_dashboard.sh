#!/bin/bash
# Start simple HTTP server for dashboard

cd /home/mazhar/final_stand/vrx/dashboard

echo "=========================================="
echo "🚤 Starting Dashboard Server"
echo "=========================================="
echo ""
echo "Dashboard available at: http://localhost:8000/simple_dashboard.html"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python3 -m http.server 8000

