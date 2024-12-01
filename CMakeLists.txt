cmake_minimum_required(VERSION 3.16)

SET(VCPKG_BOOTSTRAP_OPTIONS "-disableMetrics")    # Disable telemetry for vcpkg.
SET(X_VCPKG_APPLOCAL_DEPS_INSTALL ON)             # Install vcpkg dependencies automatically(experimental - might be changed or removed later; see: https://github.com/microsoft/vcpkg/issues/1653). 

# Forbid in-source builds.
IF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
	MESSAGE(SEND_ERROR "In-source builds are not allowed. Use a different build directory.")
ENDIF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")

# builds the crossforge library
include("crossforge/CrossForgeBuild.cmake")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /LTCG}")

# incremental linking
#set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "/INCREMENTAL:YES")
#set(CMAKE_SHARED_LINKER_FLAGS "/INCREMENTAL:YES")

#TODO(skade) time build
#if(CMAKE_GENERATOR MATCHES "Ninja")
#	set(CMAKE_NINJA_FLAGS "-d explain -d timing")
#endif()

# create required directories if non existent 
file(MAKE_DIRECTORY "MyAssets")
file(MAKE_DIRECTORY "Prototypes")
file(MAKE_DIRECTORY "Prototypes/Shader")

#################################################################
########### Project - CrossForge Sandbox ########################
#################################################################
project (CForgeSandbox)

option(INCLUDE_OPENCV "Include OpenCV in build" OFF)


if(EMSCRIPTEN)
	include(FetchContent)
	
	### Eigen3
	FetchContent_Declare(
		tinyxml2 
		GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
		GIT_TAG 9.0.0
	)
	FetchContent_MakeAvailable(tinyxml2)

	# nlohmann-json
	FetchContent_Declare(
		nlohmann_json
		GIT_REPOSITORY https://github.com/nlohmann/json.git
	)
	FetchContent_MakeAvailable(nlohmann_json)

	# tinygltf
	FetchContent_Declare(
		tinygltf 
		URL https://github.com/syoyo/tinygltf/archive/refs/tags/v2.8.2.zip
	)
	FetchContent_MakeAvailable(tinygltf)
	include_directories(${tinygltf_SOURCE_DIR}/)

	FetchContent_Declare(
		imgui
		URL https://github.com/ocornut/imgui/archive/refs/tags/v1.89.4.zip
	)
	FetchContent_MakeAvailable(imgui)
	include_directories(${imgui_SOURCE_DIR}/)
	include_directories(${imgui_SOURCE_DIR}/backends)


	set(CMAKE_EXECUTABLE_SUFFIX .html)
else()
# Additional packages not alreday included in the core library
FIND_PACKAGE(tinyxml2 CONFIG REQUIRED)	# support for xml loading, storing, and processing
FIND_PACKAGE(nlohmann_json CONFIG REQUIRED) # support for json loading, storing, and processing
FIND_PACKAGE(imgui CONFIG REQUIRED) # dear-imgui
find_package(imguizmo  CONFIG REQUIRED)
FIND_PACKAGE(JPEG REQUIRED)
FIND_PACKAGE(libjpeg-turbo)
FIND_PACKAGE(FFMPEG REQUIRED)


endif()

add_executable(CForgeSandbox 
	SandboxMain.cpp
	

	# Hardware stuff (used with Raspberry Pi)
	crossforge/Core/SGPIO.cpp
	Prototypes/Hardware/I2C.cpp 
	Prototypes/Hardware/MPU6050.cpp
	Prototypes/Hardware/RotaryEncoder.cpp

	# Camera related
	Prototypes/Camera/PhotoFinish.cpp
	Prototypes/Camera/StripPhoto.cpp 
	Prototypes/Camera/VideoPlayer.cpp
	Prototypes/Camera/CameraCapture.cpp
	Prototypes/Multimedia/SMediaDeviceManager.cpp
	Prototypes/Multimedia/CameraDevice.cpp
	Prototypes/Multimedia/FFMPEG.cpp
	
	# Misc
	Prototypes/Assets/GLTFIO/GLTFIOutil.cpp
	Prototypes/Assets/GLTFIO/GLTFIOread.cpp
	Prototypes/Assets/GLTFIO/GLTFIOwrite.cpp
	Prototypes/Assets/GLTFIO/GLTFIO.cpp
	Prototypes/MeshProcessing/SurfaceSampler.cpp
	
	Prototypes/GUI/ImGuiUtility.cpp

	# Subprojects
	# B02 Demonstrator
	Subprojects/B02Demonstrator/AdaptiveSkeletalActor.cpp
	# Dual IMU
	Subprojects/DualIMU/IMUCameraController.cpp

	# Other
	Prototypes/Actor/LODActor.cpp
	Prototypes/Actor/SLOD.cpp
	Prototypes/Actor/LODHandler.cpp
	Prototypes/Actor/UBOInstancedData.cpp
	Prototypes/MeshProcessing/MeshDecimate.cpp

	Prototypes/SkeletonConvertion.cpp

	#Inverse Kinematics Prototype
	"Prototypes/MotionRetarget/MotionRetargetScene.cpp"
	"Prototypes/MotionRetarget/MotionRetargetUI.cpp"

	"Prototypes/MotionRetarget/CharEntity.cpp"

	"Prototypes/MotionRetarget/IK/IKController.cpp"
	"Prototypes/MotionRetarget/IK/IKSkeletalActor.cpp"

	"Prototypes/MotionRetarget/IK/Solver/CCDSolver.cpp"
	"Prototypes/MotionRetarget/IK/Solver/FABRIKSolver.cpp"
	"Prototypes/MotionRetarget/IK/IKArmature.cpp"

	Prototypes/MotionRetarget/CMN/EigenMesh.cpp
	"Prototypes/MotionRetarget/CMN/Picking.cpp"
	
	Prototypes/MotionRetarget/UI/Guizmo.cpp
	Prototypes/MotionRetarget/UI/EditCamera.cpp
	Prototypes/MotionRetarget/Config/Config.cpp

	Prototypes/MotionRetarget/Animation/JointPickable.cpp

	Prototypes/MotionRetarget/AutoMoRe/MRlimb.cpp

	#Prototypes/MotionRetarget/JointLimits/JointLimits.cpp
	#Prototypes/MotionRetarget/JointLimits/HingeLimits.cpp
	#Prototypes/MotionRetarget/JointLimits/SwingXZTwistYLimits.cpp
	#Prototypes/MotionRetarget/JointLimits/SwingXTwistYLimits.cpp
	#Prototypes/MotionRetarget/JointLimits/SwingZTwistYLimits.cpp
	#Prototypes/MotionRetarget/JointLimits/SwingXYTwistZLimits.cpp

	Prototypes/MotionRetarget/CMN/EigenFWD.cpp
	"Prototypes/MotionRetarget/CMN/MRMutil.cpp"
	"Prototypes/MotionRetarget/UI/LineBox.cpp"
	"Prototypes/MotionRetarget/UI/EditGrid.cpp"
	 "Prototypes/MotionRetarget/IK/Solver/JacInvSolver.cpp"
	 #"Prototypes/MotionRetarget/IK/Solver/AnalyticSolver.cpp"

	 "Prototypes/MotionRetarget/CMN/MergeVertices.cpp"
	 "Prototypes/MotionRetarget/AutoRig/ARrignet.cpp"

	 "Prototypes/objImport/objImport.cpp"
 )

#TODO(skade) optimize gltfio TODO debug
#string(REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
#target_compile_options(CForgeSandbox PRIVATE "${CMAKE_CXX_FLAGS_DEBUG}")

#set_source_files_properties(Prototypes/Assets/GLTFIO/GLTFIOutil.cpp PROPERTIES COMPILE_FLAGS -O2)
#set_source_files_properties(Prototypes/Assets/GLTFIO/GLTFIOread.cpp PROPERTIES COMPILE_FLAGS -O2)
#set_source_files_properties(Prototypes/Assets/GLTFIO/GLTFIOwrite.cpp PROPERTIES COMPILE_FLAGS -O2)
#set_source_files_properties(Prototypes/Assets/GLTFIO/GLTFIO.cpp PROPERTIES COMPILE_FLAGS -O2)

#icon windows only
if (WIN32)
	target_sources(CForgeSandbox PRIVATE Assets/icon/icon.rc)
endif()

#TODOf(skade) get rid of warnings
#target_compile_options(CForgeSandbox PRIVATE
#	$<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
#	$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>
#)

if(EMSCRIPTEN)
	target_link_Libraries(CForgeSandbox
		tinyxml2
		nlohmann_json
		crossforge
	)
	set_target_properties(CForgeSandbox PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${Optimization_Flag} -sEXIT_RUNTIME=1 -sALLOW_MEMORY_GROWTH=1 -sWASM=1 -sUSE_WEBGL2=1 -fwasm-exceptions -sUSE_GLFW=3 -sUSE_ZLIB=1 -sUSE_LIBPNG=1 -sUSE_LIBJPEG=1 --preload-file Assets --preload-file Shader --preload-file MyAssets")

elseif(WIN32)
	target_include_directories(CForgeSandbox 
		PRIVATE ${JPEG_INCLUDE_DIR}
		PRIVATE ${FFMPEG_INCLUDE_DIRS}
	)

	target_link_directories(CForgeSandbox
		PRIVATE ${FFMPEG_LIBRARY_DIRS}
	)

	#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}") # -bigobj") # necessary?
	if (MSVC)
		#TODO(skade)
		#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /d1reportTime")
		set_source_files_properties(
			#Prototypes/MotionRetarget/MotionRetargetScene.cpp
			#"Prototypes/MotionRetarget/IK/Solver/JacInvSolver.cpp"
			PROPERTIES COMPILE_OPTIONS /d1reportTime
		)
	endif()
	target_link_libraries(CForgeSandbox 
		PRIVATE crossforge
		PRIVATE Pinocchio
		PRIVATE glfw 
		PRIVATE glad::glad
		ws2_32
		PRIVATE tinyxml2::tinyxml2
		PRIVATE nlohmann_json::nlohmann_json
		PRIVATE imgui::imgui
		PRIVATE imguizmo::imguizmo
		PRIVATE ${FFMPEG_LIBRARIES}
		${FREETYPE_LIBRARIES}
	)
	
elseif(__arm__)
	target_link_libraries(CForgeSandbox 
		PRIVATE crossforge
		PRIVATE glfw
		PRIVATE glad::glad
		PRIVATE tinyxml2::tinyxml2
		PRIVATE nlohmann_json::nlohmann_json
		PRIVATE imgui::imgui
#		PRIVATE assimp
		PRIVATE gpiod
		PRIVATE dl
		)
elseif(UNIX)
	target_include_directories(CForgeSandbox 
	PRIVATE ${JPEG_INCLUDE_DIR}
	PRIVATE ${FFMPEG_INCLUDE_DIRS}
	)

	target_link_directories(CForgeSandbox
		PRIVATE ${FFMPEG_LIBRARY_DIRS}
	)

	target_link_libraries(CForgeSandbox 
		PRIVATE crossforge
		PRIVATE glfw
		PRIVATE glad::glad
		PRIVATE tinyxml2::tinyxml2
		PRIVATE nlohmann_json::nlohmann_json
		PRIVATE imgui::imgui
		PRIVATE dl
		PRIVATE ${FFMPEG_LIBRARIES}
		${FREETYPE_LIBRARIES}
		${OpenCV_LIBS}
		)
endif()

#copy shaders
add_custom_command(
	TARGET CForgeSandbox PRE_BUILD 
	COMMAND ${CMAKE_COMMAND} -E copy_directory
	${CMAKE_SOURCE_DIR}/crossforge/Shader
	${CMAKE_CURRENT_BINARY_DIR}/Shader
)
add_custom_command(
	TARGET CForgeSandbox PRE_BUILD 
	COMMAND ${CMAKE_COMMAND} -E copy_directory 
	${CMAKE_SOURCE_DIR}/Prototypes/Shader 
	${CMAKE_CURRENT_BINARY_DIR}/Shader
)

#copy assets
add_custom_command(
	TARGET CForgeSandbox PRE_BUILD 
	COMMAND ${CMAKE_COMMAND} -E copy_directory 
	${CMAKE_SOURCE_DIR}/Assets
	${CMAKE_CURRENT_BINARY_DIR}/Assets
)

add_custom_command(
	TARGET CForgeSandbox PRE_BUILD 
	COMMAND ${CMAKE_COMMAND} -E copy_directory 
	${CMAKE_SOURCE_DIR}/MyAssets
	${CMAKE_CURRENT_BINARY_DIR}/MyAssets
)


if(EMSCRIPTEN)
set(APP_FILES CForgeSandbox.html CForgeSandbox.js CForgeSandbox.wasm CForgeSandbox.data)
foreach(i ${APP_FILES})
add_custom_command(
	TARGET CForgeSandbox POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy 
	${CMAKE_CURRENT_BINARY_DIR}/${i}
	${CMAKE_CURRENT_BINARY_DIR}/App/${i}
)
endforeach(i)
endif(EMSCRIPTEN)

add_library(Pinocchio SHARED
	Thirdparty/Pinocchio/refinement.cpp
	Thirdparty/Pinocchio/attachment.cpp
	Thirdparty/Pinocchio/pinocchioApi.cpp
	Thirdparty/Pinocchio/Pinocchio.cpp
	Thirdparty/Pinocchio/discretization.cpp
	Thirdparty/Pinocchio/mesh.cpp
	Thirdparty/Pinocchio/embedding.cpp
	Thirdparty/Pinocchio/graphutils.cpp
	Thirdparty/Pinocchio/matrix.cpp
	Thirdparty/Pinocchio/lsqSolver.cpp
	Thirdparty/Pinocchio/indexer.cpp
	Thirdparty/Pinocchio/intersector.cpp
	Thirdparty/Pinocchio/skeleton.cpp
	
	Thirdparty/Pinocchio/PinocchioTools.cpp
)
target_include_directories(Pinocchio PUBLIC "${CMAKE_SOURCE_DIR}/Thirdparty/Pinocchio")
add_compile_definitions(PINOCCHIO_EXPORTS)
target_link_libraries(Pinocchio
	PRIVATE Eigen3::Eigen
	PRIVATE crossforge
)

if (WIN32)
	# disable msvc runtime checks for o2
	# https://learn.microsoft.com/de-de/cpp/build/reference/rtc-run-time-error-checks?view=msvc-170
	# https://learn.microsoft.com/de-de/cpp/preprocessor/runtime-checks?view=msvc-170
	string(REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
	target_compile_options(Pinocchio PRIVATE "${CMAKE_CXX_FLAGS_DEBUG}")
	target_compile_options(Pinocchio PRIVATE /O2)
endif (WIN32)

#TODO(skade) time translation unit compile time
# Set global property (all targets are impacted)
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")
set_property(TARGET CForgeSandbox PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")

target_precompile_headers(CForgeSandbox PRIVATE
	pch.h
)

