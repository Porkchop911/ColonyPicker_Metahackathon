@echo off
rem Colony picker page with no robot: saved dish photos, saved SAM 3.1 detections, Muse Spark critic. Close this window to stop.
cd /d "%~dp0"
python -c "import cv2, numpy" 2>nul || python -m pip install opencv-python numpy
python tools\offline_page.py
if errorlevel 1 pause
