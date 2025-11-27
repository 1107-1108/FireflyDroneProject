idf_component_register(SRCS "FireflyDrone_main.c" "mpu6050.c" "AttitudeCtrl.c" "quaternion.c" "KalmanFilter.c"
                    INCLUDE_DIRS "." 
                    REQUIRES driver freertos)
