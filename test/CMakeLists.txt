function(make_test name)
    project(${name})
    source_group("main" FILES ${name}.cpp)
    add_executable(${name} ${name}.cpp)
    set_target_properties(${name} PROPERTIES FOLDER "test")
endfunction()

make_test(test_basic)

