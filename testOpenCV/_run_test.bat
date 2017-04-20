@echo off
set /a "x = 1"
set /p fram="Enter maximum frames : "

:while1

if %x% leq %fram% (

    echo Running frame test %x% of %fram%..
    testOpenCV.exe -f=%x% -s=false
    set /a "x = x + 1"
    goto :while1
)

endlocal