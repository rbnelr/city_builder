
@echo.
@echo.
@echo ======================== just project specific code ======================
C:\apps\cloc.exe  src shaders --exclude-dir=engine --by-file

@echo.
@echo.
@echo ======================== framework code ==================================
C:\apps\cloc.exe src/engine --exclude-dir=dear_imgui,glad,kisslib,tracy,ankerl,dds_image --by-file

@echo.
@echo.
@echo ======================== kisslib =========================================
C:\apps\cloc.exe src/engine/kisslib --exclude-dir=nlohmann_json,smhasher,stb,output --by-file 

@echo.
@echo.
@echo ======================== kissmath generated files ========================
C:\apps\cloc.exe src/engine/kisslib/kissmath/output --by-file

pause
