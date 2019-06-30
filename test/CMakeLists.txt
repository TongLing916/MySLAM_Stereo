SET(TEST_SOURCES test_triangulation)

FOREACH (test_src ${TEST_SOURCES})
    ADD_EXECUTABLE(${test_src} ${test_src}.cc)
    TARGET_LINK_LIBRARIES(${test_src} ${THIRD_PARTY_LIBS} myslam)
    ADD_TEST(NAME ${test_src} COMMAND $<TARGET_FILE:${test_src}>)
ENDFOREACH (test_src)
