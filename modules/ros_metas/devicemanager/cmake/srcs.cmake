#========================
# libs
#========================

set(CUR_SRCS "")
set(CUR_INCLUDES "include")

set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)
LIST(APPEND CUR_SUB_DIR rs_driver)

foreach (dir ${CUR_SUB_DIR})
    file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h)
    list(APPEND CUR_SRCS ${tmp_srcs})
endforeach ()

add_library(${CUR_LIB} SHARED
        ${CUR_SRCS}
        )

target_include_directories(${CUR_LIB}
        PUBLIC
        ${CUR_INCLUDES}
        ./
        ${CMAKE_CURRENT_BINARY_DIR}/../third_party/libuvc
        )
target_link_libraries(${CUR_LIB}
        PUBLIC
        usb-static 
        uvc-static  
        )

set(enable_test false)
if(enable_test)
        message("enable device test !")
        add_executable(devicemanager_test ./test/devicemanager_test.cpp)
        target_link_libraries(devicemanager_test device)

        add_executable(devicemanager_test2 ./test/devicemanager_test2.cpp)
        target_link_libraries(devicemanager_test2 device)

        add_executable(devicemanager_test3 ./test/devicemanager_test3.cpp)
        target_link_libraries(devicemanager_test3 device)
else() 
        message("disable device test !")
endif(enable_test) 

#=============================
# install
#=============================

install(TARGETS ${CUR_LIB}
        LIBRARY DESTINATION ${HYPER_VISION_DEV_LIB_PATH}
        ARCHIVE DESTINATION ${HYPER_VISION_DEV_LIB_PATH}
        COMPONENT release
        )
