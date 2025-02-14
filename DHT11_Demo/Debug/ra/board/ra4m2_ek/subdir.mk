################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ra/board/ra4m2_ek/board_init.c \
../ra/board/ra4m2_ek/board_leds.c 

C_DEPS += \
./ra/board/ra4m2_ek/board_init.d \
./ra/board/ra4m2_ek/board_leds.d 

OBJS += \
./ra/board/ra4m2_ek/board_init.o \
./ra/board/ra4m2_ek/board_leds.o 

SREC += \
DHT11_Demo.srec 

MAP += \
DHT11_Demo.map 


# Each subdirectory must supply rules for building sources it contributes
ra/board/ra4m2_ek/%.o: ../ra/board/ra4m2_ek/%.c
	$(file > $@.in,-mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -O2 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-strict-aliasing -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal -g -D_RENESAS_RA_ -D_RA_CORE=CM33 -D_RA_ORDINAL=1 -I"C:/Users/User/e2_studio/workspace/DHT11_Demo/src" -I"." -I"C:/Users/User/e2_studio/workspace/DHT11_Demo/ra/fsp/inc" -I"C:/Users/User/e2_studio/workspace/DHT11_Demo/ra/fsp/inc/api" -I"C:/Users/User/e2_studio/workspace/DHT11_Demo/ra/fsp/inc/instances" -I"C:/Users/User/e2_studio/workspace/DHT11_Demo/ra/arm/CMSIS_6/CMSIS/Core/Include" -I"C:/Users/User/e2_studio/workspace/DHT11_Demo/ra_gen" -I"C:/Users/User/e2_studio/workspace/DHT11_Demo/ra_cfg/fsp_cfg/bsp" -I"C:/Users/User/e2_studio/workspace/DHT11_Demo/ra_cfg/fsp_cfg" -std=c99 -Wno-stringop-overflow -Wno-format-truncation --param=min-pagesize=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" -x c "$<")
	@echo Building file: $< && arm-none-eabi-gcc @"$@.in"

