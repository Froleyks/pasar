cmake_minimum_required (VERSION 2.8)
set(CMAKE_BUILD_TYPE Debug)
include_directories(.)

set (RUN_PARAMS
  -v 7 -s 3 /home/froleyks/Dropbox/mt/domains_agile/Tetris/p022.pddl.sas
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
  -Weffc++
  -Wunreachable-code
  )

string (REPLACE ";" " " WARNINGS_STR "${WARNINGS}")
set(CMAKE_CXX_FLAGS "-std=c++17 -msse4.2 -g -O3 -march=native ${WARNINGS_STR}")

message(${CMAKE_CXX_FLAGS})

add_custom_command(OUTPUT problem.plan
  COMMAND ./ADAL_glucose ${RUN_PARAMS} -planfile problem.plan
  COMMAND ./../validate ${RUN_PARAMS} problem.plan
  COMMAND rm problem.plan
  DEPENDS ADAL_glucose
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
