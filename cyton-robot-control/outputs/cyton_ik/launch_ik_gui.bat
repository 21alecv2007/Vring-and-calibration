@echo off
cd /d "%~dp0"
if exist "..\..\.venv\Scripts\pythonw.exe" (
  "..\..\.venv\Scripts\pythonw.exe" cyton_ik_gui.py %*
) else (
  pythonw cyton_ik_gui.py %*
)
