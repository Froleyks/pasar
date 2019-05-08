cmake_minimum_required (VERSION 2.8)
set(CMAKE_BUILD_TYPE Release)
include_directories(.)

set (RUN_PARAMS
  -v 7
  )

set (SOURCE_FILES
  src/main.cpp
  src/problem/problem.cpp
  )

set (CMAKE_CXX_FLAGS "-std=c++17 -msse4.2 -g -O3 -march=native")

# glucose
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/sat/glucose4/libipasirglucose4.a
  COMMAND make all
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/sat/glucose4
  )
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/sat/glucose4)
add_executable(ADAL_glucose ${SOURCE_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/sat/glucose4/libipasirglucose4.a)
target_link_libraries(ADAL_glucose ipasirglucose4.a)
add_custom_target(run_glucose
  ./ADAL_glucose ${RUN_PARAMS}
  DEPENDS ADAL_glucose
)

# minisat
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/sat/minisat220/libipasirminisat220.a
  COMMAND make all
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/sat/minisat220
  )
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/sat/minisat220)
add_executable(ADAL_minisat ${SOURCE_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/sat/minisat220/libipasirminisat220.a)
target_link_libraries(ADAL_minisat ipasirminisat220.a)
add_custom_target(run_minisat
  ./ADAL_minisat ${RUN_PARAMS}
  DEPENDS ADAL_minisat
)

# lingeling
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/sat/lingeling/liblgl.a
  COMMAND ./configure.sh
  COMMAND make all
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/sat/lingeling
  )
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/sat/lingeling)
add_executable(ADAL_lingeling ${SOURCE_FILES} sat/lingeling/Lingeling.cpp ${CMAKE_CURRENT_SOURCE_DIR}/sat/lingeling/liblgl.a)
target_link_libraries(ADAL_lingeling lgl.a)
target_compile_options(ADAL_lingeling PRIVATE "-llgl")
add_custom_target(run_lingeling
  ./ADAL_lingeling ${RUN_PARAMS}
  DEPENDS ADAL_lingeling
)

# picosat
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/sat/picosat961/libipasirpicosat961.a
  COMMAND make all
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/sat/picosat961
  )
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/sat/picosat961)
add_executable(ADAL_picosat ${SOURCE_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/sat/picosat961/libipasirpicosat961.a)
target_link_libraries(ADAL_picosat ipasirpicosat961.a)
add_custom_target(run_picosat
  ./ADAL_picosat ${RUN_PARAMS}
  DEPENDS ADAL_picosat
)

add_compile_options(${Warnings})
set(Warnings
  -Wall
  -Wextra # reasonable and standard
  -Wshadow # warn the user if a variable declaration shadows one from a
  # parent context
  -Wnon-virtual-dtor # warn the user if a class with virtual functions has a
  # non-virtual destructor. This helps catch hard to
  # track down memory errors
  -Wold-style-cast # warn for c-style casts
  -Wcast-align # warn for potential performance problem casts
  -Wunused # warn on anything being unused
  -Woverloaded-virtual # warn if you overload (not override) a virtual
  # function
  -Wpedantic # warn if non-standard C++ is used
  -Wconversion # warn on type conversions that may lose data
  -Wsign-conversion # warn on sign conversions
  -Wmisleading-indentation # warn if identation implies blocks where blocks
  # do not exist
  -Wduplicated-cond # warn if if / else chain has duplicated conditions
  -Wduplicated-branches # warn if if / else branches have duplicated code
  -Wlogical-op # warn about logical operations being used where bitwise were
  # probably wanted
  -Wnull-dereference # warn if a null dereference is detected
  -Wuseless-cast # warn if you perform a cast to the same type
  -Wdouble-promotion # warn if float is implicit promoted to double
  -Wformat=2 # warn on security issues around functions that format output
  -Weffc++
  -Wunreachable-code
  )
