@echo off

if not exist ..\build mkdir ..\build

set CFLAGS=/nologo /Od /Zi /EHsc
set LIBS=OgreBites.lib OgreRTShaderSystem.lib OgreMain.lib
set INC_DIR=/I"D:\Libs\ogre\sdk\include\OGRE" /I"D:\Libs\ogre\sdk\include\OGRE\Bites" /I"D:\Libs\ogre\sdk\include\OGRE\RTShaderSystem"
set LNK_DIR=/LIBPATH:"D:\Libs\ogre\sdk\lib"

pushd ..\build

    cl %CFLAGS% %INC_DIR% ..\src\*.cpp /Fe.\game /link %LNK_DIR% %LIBS%

popd
