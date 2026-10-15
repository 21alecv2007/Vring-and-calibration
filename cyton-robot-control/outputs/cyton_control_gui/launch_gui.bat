@echo off
cd /d "%~dp0"
if exist "..\..\.venv\Scripts\pythonw.exe" (
  "..\..\.venv\Scripts\pythonw.exe" cyton_control_gui.py
) else (
  pythonw cyton_control_gui.py
)
