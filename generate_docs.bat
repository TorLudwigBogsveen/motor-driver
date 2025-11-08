@echo off
REM Generate Doxygen Documentation for CAN Motor Controller Simulator
REM Created by GitHub Copilot

echo Generating documentation with Doxygen...
"Z:\Program Files\doxygen\bin\doxygen.exe" Doxyfile.custom

if %ERRORLEVEL% == 0 (
    echo.
    echo ✅ Documentation generated successfully!
    echo Opening documentation in browser...
    start docs\html\index.html
) else (
    echo.
    echo ❌ Error generating documentation
    pause
)