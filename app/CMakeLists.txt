include_directories ("${PROJECT_SOURCE_DIR}/app")

# Adds sources to the Solution Explorer
add_executable (evca_encoder evca_encoder.c)
add_executable (evca_decoder evca_decoder.c)
add_executable (evca_bitstream_merge evca_bitstream_merge.c)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Properties->Linker->Input->Additional Dependencies
target_link_libraries (evca_encoder evc_encoder_lib)
target_link_libraries (evca_decoder evc_decoder_lib)
target_link_libraries (evca_bitstream_merge evc_encoder_lib)
target_link_libraries (evca_bitstream_merge evc_decoder_lib)

# Creates a folder "executables" and adds target 
# project (app.vcproj) under it
set_property(TARGET evca_encoder PROPERTY FOLDER "app")
set_property(TARGET evca_decoder PROPERTY FOLDER "app")
set_property(TARGET evca_bitstream_merge PROPERTY FOLDER "app")

set_target_properties(evca_encoder PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set_target_properties(evca_decoder PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set_target_properties(evca_bitstream_merge PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

target_include_directories( evca_bitstream_merge PUBLIC . .. ../inc/ ../src/dec ../src/enc)

if( MSVC )                      
    target_compile_definitions( evca_encoder PUBLIC _CRT_SECURE_NO_WARNINGS )
    target_compile_definitions( evca_decoder PUBLIC _CRT_SECURE_NO_WARNINGS )
    target_compile_definitions( evca_bitstream_merge PUBLIC _CRT_SECURE_NO_WARNINGS )
endif()

