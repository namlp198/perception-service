@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "DEPLOY_ARGUMENTS=%*"

echo ============================================================
echo perception-service - Jetson rsync deployment
echo Target default: percpc@192.168.1.220
echo ============================================================

if "%~1"=="" (
    echo.
    echo Select deployment mode:
    echo   [1] Dry-run only - show files, make no changes
    echo   [2] Stage update - sync, build and test; do not restart [recommended]
    echo   [3] Full deploy - sync, build, test and restart service
    echo   [4] Source sync only - do not build or restart
    echo   [Q] Cancel
    echo.
    "%SystemRoot%\System32\choice.exe" /c 1234Q /n /m "Select [1/2/3/4/Q]: "

    if errorlevel 5 goto :cancelled
    if errorlevel 4 goto :source_only
    if errorlevel 3 goto :full_deploy
    if errorlevel 2 goto :stage_update
    goto :dry_run
)

goto :run_deploy

:dry_run
set "DEPLOY_ARGUMENTS=-InteractiveAuth -DryRun"
goto :run_deploy

:full_deploy
set "DEPLOY_ARGUMENTS=-InteractiveAuth -Build -Test -Restart"
goto :run_deploy

:stage_update
set "DEPLOY_ARGUMENTS=-InteractiveAuth -Build -Test"
goto :run_deploy

:source_only
set "DEPLOY_ARGUMENTS=-InteractiveAuth"
goto :run_deploy

:run_deploy
echo.
echo Arguments: %DEPLOY_ARGUMENTS%
echo.

"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%deploy.ps1" -DetailedLog %DEPLOY_ARGUMENTS%
set "DEPLOY_EXIT_CODE=%ERRORLEVEL%"

if "%DEPLOY_EXIT_CODE%"=="3" (
    echo.
    echo Deployment finished with IMU ACCEPTANCE FAILED ^(exit code 3^).
    echo Service restarted and RGB/depth RTSP are live, but accel/gyro data is missing.
    echo IMU data is mandatory before EKF/mission use; see docs\operations.md.
) else if not "%DEPLOY_EXIT_CODE%"=="0" (
    echo.
    echo Deployment FAILED with exit code %DEPLOY_EXIT_CODE%.
) else (
    echo.
    echo Deployment completed successfully.
)

echo.
echo Detailed log files are stored under:
echo %SCRIPT_DIR%..\..\out\deploy-logs

if /i not "%PERCEPTION_DEPLOY_NO_PAUSE%"=="1" (
    echo.
    echo Press any key to close this window...
    pause >nul
)

exit /b %DEPLOY_EXIT_CODE%

:cancelled
echo.
echo Deployment cancelled. No changes were made.
echo.
echo Press any key to close this window...
pause >nul
exit /b 0
