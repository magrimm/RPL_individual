################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/object_recognition_node.cpp \
../src/pointcloud_publisher.cpp \
../src/pointcloud_subscriber.cpp 

OBJS += \
./src/object_recognition_node.o \
./src/pointcloud_publisher.o \
./src/pointcloud_subscriber.o 

CPP_DEPS += \
./src/object_recognition_node.d \
./src/pointcloud_publisher.d \
./src/pointcloud_subscriber.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


