@echo off
rem Colony picker demo page (tools/demo_server.py) with the robot's Python (it has sam3 for colony detection).
cd /d "%~dp0"
start "" "D:\robot\highwonder_rework\.worktrees\hackathon\.venv\Scripts\pythonw.exe" tools\demo_server.py
timeout /t 2 /nobreak >nul
start "" http://127.0.0.1:8765
