@echo off
rem fault_tests.bat - Task 4: what happens when a wall reading is wrong.
rem Builds solver.exe from solver.c, then runs it in mm_check.py, which plays
rem the simulator and gives the solver one wrong wall reading per test.
rem The report is shown and saved to fault_tests.txt.
gcc -O2 -std=c99 -o solver.exe main_sim.c hal_sim.c mm_common.c solver.c
if errorlevel 1 goto buildfail
(
echo === Test 1: the sensor misses a wall, and the mouse bumps into it
python mm_check.py solver.exe mazes\practice8_1.txt --miss 0,1,N
echo.
echo === Test 2: the sensor misses a wall that the next cell saw correctly
python mm_check.py solver.exe mazes\practice8_1.txt --miss 3,1,W
echo.
echo === Test 3: the sensor misses a wall the mouse never needs to cross
python mm_check.py solver.exe mazes\practice8_1.txt --miss 3,4,W
echo.
echo === Test 4: the sensor sees a wall that is not there, on the only way to the goal
python mm_check.py solver.exe mazes\practice8_1.txt --phantom 1,1,E
echo.
echo === Test 5: the same false wall, seen only once
python mm_check.py solver.exe mazes\practice8_1.txt --phantom 1,1,E --once
echo.
echo === Test 6: the operator presses Reset twice during the run
python mm_check.py solver.exe mazes\practice16_1.txt --reset-at 40,200
) > fault_tests.txt 2>&1
type fault_tests.txt
goto end
:buildfail
echo Build failed: check that solver.c, main_sim.c, hal_sim.c and mm_common.c are here.
:end
