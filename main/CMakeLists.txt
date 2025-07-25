idf_component_register(SRCS "rainsensor_main.c"
                    REQUIRES E32_Lora_Lib 
                    INCLUDE_DIRS "."
                    REQUIRES driver soc nvs_flash ulp)
#
# ULP support additions to component CMakeLists.txt.
#
# 1. The ULP app name must be unique (if multiple components use ULP).
set(ulp_app_name ulp_${COMPONENT_NAME})
#
# 2. Specify all assembly source files.
#    Files should be placed into a separate directory (in this case, ulp/),
#    which should not be added to COMPONENT_SRCS.
set(ulp_s_sources "ulp/ulp_trigger_cnt.S" "ulp/wake_up.S")
#
# 3. List all the component source files which include automatically
#    generated ULP export file, ${ulp_app_name}.h:
set(ulp_exp_dep_srcs "rainsensor_main.c")
message(STATUS "AppName: ${ulp_app_name}")
message(STATUS "ComponentName: ${COMPONENT_NAME}")

#
# 4. Call function to build ULP binary and embed in project using the argument
#    values above.
ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
