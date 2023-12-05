cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(ComputerGraphics C CXX)

option(USE_PREBUILT_INTERSECT "Enable using prebuilt intersection library" OFF)

if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/framework")
	# Create framework library and include CMake scripts (compiler warnings, sanitizers and static analyzers).
	add_subdirectory("framework")
	set(REFERENCE_MODE OFF)
else()
	# During development the framework lives in parent folder.
	add_subdirectory("../../../framework/" "${CMAKE_BINARY_DIR}/framework/")
	set(REFERENCE_MODE ON)
endif()

find_package(OpenGL REQUIRED)
find_package(OpenMP REQUIRED)

add_library(CGFinProjLib
	"src/bounding_volume_hierarchy.cpp"
	"src/bvh_interface.cpp"
	"src/config.cpp"
	"src/draw.cpp"
	"src/interpolate.cpp"
	"src/light.cpp"
	"src/render.cpp"
	"src/reservoir.cpp"
	"src/scene.cpp"
	"src/screen.cpp"
	"src/shading.cpp"
	"src/texture.cpp"
	"src/tone_mapping.cpp"
	"src/ui.cpp"
	"src/utils.cpp"
)

if (REFERENCE_MODE)
	target_sources(CGFinProjLib PRIVATE
		"src/extra/motion_blur.cpp"
		"src/extra/multiple_rays.cpp"
		"src/extra/bloom_effect.cpp"
		"src/extra/glossy_reflection.cpp"
	)
endif()

target_include_directories(CGFinProjLib PUBLIC "src")
target_link_libraries(CGFinProjLib PUBLIC CGFramework OpenGL::GLU OpenMP::OpenMP_CXX)
target_compile_features(CGFinProjLib PUBLIC cxx_std_20)
enable_sanitizers(CGFinProjLib)
set_project_warnings(CGFinProjLib)

if (USE_PREBUILT_INTERSECT)
	add_library(Intersect2 STATIC IMPORTED)
	if (WIN32)
		set_target_properties(Intersect2 PROPERTIES IMPORTED_LOCATION_DEBUG "${CMAKE_SOURCE_DIR}/prebuilt/Intersect_x64_debug.lib")
		set_target_properties(Intersect2 PROPERTIES IMPORTED_LOCATION_RELEASE "${CMAKE_SOURCE_DIR}/prebuilt/Intersect_x64_release.lib")
	elseif (APPLE)
		set_target_properties(Intersect2 PROPERTIES IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/prebuilt/libIntersect_mac_arm64.a")
	else()
		set_target_properties(Intersect2 PROPERTIES IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/prebuilt/libIntersect_linux_x64.a")
	endif()
	target_link_libraries(CGFinProjLib PUBLIC Intersect2)
else()
	target_sources(CGFinProjLib PRIVATE "src/intersect.cpp")
endif()

target_compile_definitions(CGFinProjLib PUBLIC
	"-DDATA_DIR=\"${CMAKE_CURRENT_LIST_DIR}/data/\"")

add_executable(SeminarImpl	"src/main.cpp")
target_link_libraries(SeminarImpl PUBLIC CGFinProjLib)
target_compile_features(SeminarImpl PUBLIC cxx_std_20)
enable_sanitizers(SeminarImpl)
set_project_warnings(SeminarImpl)
