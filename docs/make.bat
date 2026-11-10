@ECHO OFF

pushd %~dp0

REM Command file for Sphinx documentation

if "%SPHINXBUILD%" == "" (
	set SPHINXBUILD=sphinx-build
)
if "%PYTHON%" == "" (
	set PYTHON=python
)
set EN=source/en
set ZH=source/zh
set BUILDDIR=build
set STAGE_SCRIPT=_scripts\stage_sources.py

REM Keep Sphinx console progress in English. conf.py still controls each
REM generated site's language independently.
set LANGUAGE=en
set LC_ALL=C
set LANG=C

if /I "%1" == "clean" goto clean

%SPHINXBUILD% >NUL 2>NUL
if errorlevel 9009 (
	echo.
	echo.The 'sphinx-build' command was not found. Make sure you have Sphinx
	echo.installed, then set the SPHINXBUILD environment variable to point
	echo.to the full path of the 'sphinx-build' executable. Alternatively you
	echo.may add the Sphinx directory to PATH.
	echo.
	echo.If you don't have Sphinx installed, grab it from
	echo.https://www.sphinx-doc.org/
	exit /b 1
)

if "%1" == "" goto help

%PYTHON% %STAGE_SCRIPT% --language en
if errorlevel 1 exit /b %errorlevel%
%PYTHON% %STAGE_SCRIPT% --language zh
if errorlevel 1 exit /b %errorlevel%

set EN=build/source/en
set ZH=build/source/zh

if "%SPHINXOPTS%" == "" set SPHINXOPTS=-W --keep-going

%SPHINXBUILD% -M %1 %EN% %BUILDDIR%/en %SPHINXOPTS% %O%
if errorlevel 1 exit /b %errorlevel%
%SPHINXBUILD% -M %1 %ZH% %BUILDDIR%/zh %SPHINXOPTS% %O%
goto end

:clean
echo Cleaning generated documentation under docs\%BUILDDIR%...
echo Documentation source files will be preserved.
%PYTHON% %STAGE_SCRIPT% --clean
if errorlevel 1 goto clean_error
echo Documentation clean completed.
goto end
:clean_error
echo Documentation clean failed.
exit /b 1
:help
%PYTHON% %STAGE_SCRIPT% --language en
if errorlevel 1 exit /b %errorlevel%
%SPHINXBUILD% -M help build/source/en %BUILDDIR%/en %SPHINXOPTS% %O%

:end
popd
