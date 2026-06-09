cmake_minimum_required(VERSION 3.14.3)

if( NOT UNIX )
	#Path to SystemVue installer directory on Windows.
	set( SystemVueInstallDirectory "D:/SystemVue/")
else()
	#Path to SystemVue code generation libraries Linux.
	set( SystemVueInstallDirectory "$ENV{HOME}/SystemVue/2023")
endif()

include ( ${SystemVueInstallDirectory}/ModelBuilder/CMake/CodeGeneration.cmake )

project( ADC )

add_subdirectory( SystemVue )
