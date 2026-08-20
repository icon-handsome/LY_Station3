@echo off
rem CLion / Ninja: initialize MSVC v142 (19.29) + Windows SDK for ScanTracking presets.
call "%ProgramFiles%\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" -vcvars_ver=14.29 >nul
