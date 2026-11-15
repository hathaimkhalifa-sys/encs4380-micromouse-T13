@echo off
rem run_one.bat NAME  -  get ready to run one maze in mms and save its output.
rem NAME is the maze name without .txt, from the mazes\ or tests\ folder.
rem It copies NAME.goal to mm_goal.txt and opens results\NAME.txt in Notepad.
if "%~1"=="" goto usage
set "G="
if exist "tests\%~1.goal" set "G=tests\%~1.goal"
if exist "mazes\%~1.goal" set "G=mazes\%~1.goal"
if not defined G goto nogoal
copy /y "%G%" mm_goal.txt >nul
echo Goal for %~1 copied to mm_goal.txt:
type mm_goal.txt
if not exist results mkdir results
echo.
echo NOW IN MMS: load the maze %~1.txt, press Run and wait until the mouse stops.
echo Then click in Run Output, press Ctrl+A and Ctrl+C,
echo paste into the Notepad window that opens now, press Ctrl+S and close it.
notepad "results\%~1.txt"
goto end
:nogoal
echo No goal file %~1.goal in tests\ or mazes\. Check the name, without .txt
goto end
:usage
echo Usage: run_one NAME     for example: run_one hs6_bottom_right
:end
