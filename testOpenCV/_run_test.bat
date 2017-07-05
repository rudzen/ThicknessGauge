@echo off
set /a "x = 1"
set /p mm="Enter mm for this run : "
set /p fram="Enter runs : "

:while1

if %x% leq %fram% (

    echo Running frame test %x% of %fram%..
    ThicknessGauge.exe -d --glob_name=camera --show_windows=0 >> mm_%mm%_runs_%fram%_.txt
	REM | type mm_%mm%_runs_%fram%_.txt
    set /a "x = x + 1"
    goto :while1
)

find "diff from baseline" mm_%mm%_runs_%fram%_.txt >> mm_%mm%_runs_%fram%_results.txt

endlocal