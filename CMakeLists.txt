cmake_minimum_required (VERSION 2.8)
set(CMAKE_BUILD_TYPE Release)
include_directories(.)

set (RUN_PARAMS
  -v 7 -s 3
  )

set (SOURCE_FILES
  src/main.cpp
  src/search/greedy_best_first.cpp
  src/search/depth_first_search.cpp
  )

set (WARNINGS
  -Wall
  -Wextra
  -Wshadow
  -Wnon-virtual-dtor
  -Wold-style-cast
  -Wcast-align
  -Wunused
  -Woverloaded-virtual
  -Wpedantic
  -Wconversion
  -Wsign-conversion
  -Wmisleading-indentation
  -Wduplicated-cond
  -Wduplicated-branches
  -Wlogical-op
  -Wnull-dereference
  -Wuseless-cast
  -Wdouble-promotion
  -Wformat=2
  -Wunreachable-code
  # -Weffc++
  )

string (REPLACE ";" " " WARNINGS_STR "${WARNINGS}")
set(CMAKE_CXX_FLAGS "-std=c++17 -msse4.2 -g -O3 -march=native ${WARNINGS_STR}")

message(${CMAKE_CXX_FLAGS})

add_custom_command(OUTPUT problem.plan
  COMMAND ./pasar_glucose ${RUN_PARAMS} -planfile problem.plan
  COMMAND ./../validate ${RUN_PARAMS} problem.plan
  COMMAND rm problem.plan
  DEPENDS pasar_glucose
  )

add_custom_target(validate
  DEPENDS problem.plan
  )

# glucose
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/sat/glucose4/libipasirglucose4.a
  COMMAND make all
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/sat/glucose4
  )
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/sat/glucose4)
add_executable(pasar_glucose ${SOURCE_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/sat/glucose4/libipasirglucose4.a)
target_link_libraries(pasar_glucose ipasirglucose4.a)
add_custom_target(run_glucose
  ./pasar_glucose ${RUN_PARAMS}
  DEPENDS pasar_glucose
)

# minisat
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/sat/minisat220/libipasirminisat220.a
  COMMAND make all
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/sat/minisat220
  )
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/sat/minisat220)
add_executable(pasar_minisat ${SOURCE_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/sat/minisat220/libipasirminisat220.a)
target_link_libraries(pasar_minisat ipasirminisat220.a)
add_custom_target(run_minisat
  ./pasar_minisat ${RUN_PARAMS}
  DEPENDS pasar_minisat
)

# lingeling
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/sat/lingeling/liblgl.a
  COMMAND ./configure.sh
  COMMAND make all
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/sat/lingeling
  )
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/sat/lingeling)
add_executable(pasar_lingeling ${SOURCE_FILES} sat/lingeling/Lingeling.cpp ${CMAKE_CURRENT_SOURCE_DIR}/sat/lingeling/liblgl.a)
target_link_libraries(pasar_lingeling lgl.a)
target_compile_options(pasar_lingeling PRIVATE "-llgl")
add_custom_target(run_lingeling
  ./pasar_lingeling ${RUN_PARAMS}
  DEPENDS pasar_lingeling
)

# picosat
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/sat/picosat961/libipasirpicosat961.a
  COMMAND make all
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/sat/picosat961
  )
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/sat/picosat961)
add_executable(pasar_picosat ${SOURCE_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/sat/picosat961/libipasirpicosat961.a)
target_link_libraries(pasar_picosat ipasirpicosat961.a)
add_custom_target(run_picosat
  ./pasar_picosat ${RUN_PARAMS}
  DEPENDS pasar_picosat
)
