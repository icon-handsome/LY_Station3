@echo off
setlocal EnableExtensions

rem ScanTracking dev helper (CMakePresets + README). Used by Cursor/VS Code tasks.

for %%I in ("%~dp0..") do set "ROOT=%%~fI"
set "CMAKE=C:\Program Files\CMake\bin\cmake.exe"
set "PRESET_DEBUG=win-msvc2019-qtcore-ninja-debug"
set "PRESET_RELEASE=win-msvc2019-qtcore-ninja-release"
set "APP_DEBUG=%ROOT%\build\%PRESET_DEBUG%-s3\app"
set "APP_RELEASE=%ROOT%\build\%PRESET_RELEASE%-s3\app"

if "%~1"=="" goto usage

if /I "%~1"=="configure-debug" goto configure_debug
if /I "%~1"=="configure-release" goto configure_release
if /I "%~1"=="build-debug" goto build_debug
if /I "%~1"=="build-release" goto build_release
if /I "%~1"=="run-debug" goto run_debug
if /I "%~1"=="run-release" goto run_release
goto usage

:configure_debug
"%CMAKE%" --preset %PRESET_DEBUG% -S "%ROOT%"
exit /b %ERRORLEVEL%

:configure_release
"%CMAKE%" --preset %PRESET_RELEASE% -S "%ROOT%"
exit /b %ERRORLEVEL%

:build_debug
"%CMAKE%" --build --preset %PRESET_DEBUG%
exit /b %ERRORLEVEL%

:build_release
"%CMAKE%" --build --preset %PRESET_RELEASE%
exit /b %ERRORLEVEL%

:run_debug
call :apply_runtime_path "%APP_DEBUG%"
cd /d "%APP_DEBUG%"
if not exist "%APP_DEBUG%\scan-tracking.exe" (
  echo [scan_tracking_dev] Missing %APP_DEBUG%\scan-tracking.exe - run build-debug first
  exit /b 1
)
"%APP_DEBUG%\scan-tracking.exe" %*
exit /b %ERRORLEVEL%

:run_release
call :apply_runtime_path "%APP_RELEASE%"
cd /d "%APP_RELEASE%"
if not exist "%APP_RELEASE%\scan-tracking.exe" (
  echo [scan_tracking_dev] Missing %APP_RELEASE%\scan-tracking.exe - run build-release first
  exit /b 1
)
"%APP_RELEASE%\scan-tracking.exe" %*
exit /b %ERRORLEVEL%

:apply_runtime_path
set "APP_DIR=%~1"
set "MVS_RT=C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64"
rem 与工位一对齐：CXP GenTL Producer 路径（缺省时 EnumDevices 返回 0x800000FF）
set "GENICAM_GENTL64_PATH=%MVS_RT%;%APP_DIR%;%APP_DIR%\hik_mvs_runtime"
set "PATH=%APP_DIR%;%APP_DIR%\mech_eye_api;%APP_DIR%\ThirdParty;%APP_DIR%\hik_mvs_runtime;%MVS_RT%;C:\Qt\5.15.2\msvc2019_64\bin;%ROOT%\third_party\Mech-Eye SDK-2.5.4\API\dll;%ROOT%\third_party\Mech-Eye SDK-2.5.4\API\dll_debug;%PATH%"
echo [scan_tracking_dev] GENICAM_GENTL64_PATH=%GENICAM_GENTL64_PATH%
if not exist "%MVS_RT%\MvFGProducerCXP.cti" if not exist "%APP_DIR%\MvFGProducerCXP.cti" (
  echo [scan_tracking_dev] WARN: MvFGProducerCXP.cti not found
)
exit /b 0

:usage
echo.
echo Usage:
echo   scan_tracking_dev.cmd configure-debug ^| configure-release
echo   scan_tracking_dev.cmd build-debug     ^| build-release
echo   scan_tracking_dev.cmd run-debug       ^| run-release
echo.
exit /b 1
