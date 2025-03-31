F1_TARGETS  += $(TARGET)
FEATURES     = ONBOARDFLASH HIGHEND

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
            drivers/barometer_bmp280.c \
            hardware_revision.c

