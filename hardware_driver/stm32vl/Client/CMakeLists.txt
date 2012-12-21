SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -ggdb -DSANDBOX")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -ggdb -DSANDBOX")

INCLUDE_DIRECTORIES( ../stm32_lib/inc ../)
ADD_EXECUTABLE(testpc main.cpp UARTCommunication.cpp ComProtocol.cpp ProtocolHandler.cpp ../crc.c)
ADD_EXECUTABLE(regeln main2.cpp UARTCommunication.cpp ComProtocol.cpp ProtocolHandler.cpp ../crc.c)
target_link_libraries (testpc pthread rt)
