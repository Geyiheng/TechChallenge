# Some utilities
#TODO: comment/document

macro(standard_paths ARG0 ARG1 ARG2 ARG3)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${ARG0}/${ARG1})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${ARG0}/${ARG2})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ARG0}/${ARG2})
    set(CMAKE_PDB_OUTPUT_DIRECTORY ${ARG0}/${ARG3})
    foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
    endforeach()
    set(CMAKE_DEBUG_POSTFIX d)
    set(CMAKE_MINSIZEREL_POSTFIX min)
endmacro()

macro(standard_config)
    set(CMAKE_AUTOMOC YES)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
    set(CMAKE_CONFIGURATION_TYPES "Release")
    set(CMAKE_INCLUDE_CURRENT_DIR YES)
    IF(WIN32)
        # muti-processors build for VS
        include(ProcessorCount)
        ProcessorCount(N)
        if(N GREATER_EQUAL 16)
            set(N 16)
        endif()
        if(NOT N EQUAL 0)
            SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   /MP${N} ")
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP${N} ")
        endif()
    ENDIF()
endmacro()

macro(copy_if_not_exists FILE SRC DEST)
    IF(NOT EXISTS ${FILE})
        file(COPY ${SRC} DESTINATION ${DEST})
    ENDIF()
endmacro()
