# common library
set( LIB_NAME evc_lib )

file (GLOB ETM_INC "../inc/*.h") 
file (GLOB COMMON_LIB_FUNCTION_SRC "evc_*.c" "evc.c")
file (GLOB COMMON_LIB_FUNCTION_INC "evc_*.h" "wrapper.h")

add_library( ${LIB_NAME} STATIC ${ETM_INC} ${COMMON_LIB_FUNCTION_SRC} ${COMMON_LIB_FUNCTION_INC} )

target_include_directories( ${LIB_NAME} PUBLIC . .. ../inc/)

set_target_properties(${LIB_NAME} PROPERTIES FOLDER lib)

set( SSE ${BASE_INC_FILES} evc_mc.c evc_util.c evc_itdq.c)

if( UNIX OR MINGW )
  set_property( SOURCE ${SSE} APPEND PROPERTY COMPILE_FLAGS "-msse4.1" )
  target_link_libraries(${LIB_NAME} m)
endif()

# encdoer library
set( ENC_LIB_NAME evc_encoder_lib )

file (GLOB ENC_LIB_FUNCTION_SRC "evce*.c")
file (GLOB ENC_LIB_FUNCTION_SRC_CXX "evce*.cpp" "Enc*.cpp")
file (GLOB ENC_LIB_FUNCTION_INC "enc*.h")

include_directories (.)

add_library( ${ENC_LIB_NAME} STATIC ${ENC_LIB_FUNCTION_SRC} ${ENC_LIB_FUNCTION_SRC_CXX} ${ENC_LIB_FUNCTION_INC} )

target_compile_definitions( ${ENC_LIB_NAME} PUBLIC )
target_include_directories( ${ENC_LIB_NAME} PUBLIC . )
target_link_libraries( ${ENC_LIB_NAME} evc_lib)

set_target_properties(${ENC_LIB_NAME} PROPERTIES FOLDER lib)

set( SSE ${BASE_INC_FILES} evce_pinter.c evce_sad.c)

if( UNIX OR MINGW )
  set_property( SOURCE ${SSE} APPEND PROPERTY COMPILE_FLAGS "-msse4.2" )
endif()

# decoder library
set( DEC_LIB_NAME evc_decoder_lib )

file (GLOB DEC_LIB_FUNCTION_SRC "evcd*.c")
file (GLOB DEC_LIB_FUNCTION_INC "evcd*.h")

add_library( ${DEC_LIB_NAME} STATIC ${DEC_LIB_FUNCTION_SRC} ${DEC_LIB_FUNCTION_INC} )

target_compile_definitions( ${DEC_LIB_NAME} PUBLIC )
target_include_directories( ${DEC_LIB_NAME} PUBLIC . .. )
target_link_libraries( ${DEC_LIB_NAME} evc_lib)
if( UNIX OR MINGW )  
  target_link_libraries(${DEC_LIB_NAME} m)
endif()

set_target_properties(${DEC_LIB_NAME} PROPERTIES FOLDER lib)