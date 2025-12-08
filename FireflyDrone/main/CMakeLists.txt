idf_component_register(SRCS "pid.c" "FireflyDrone_main.c" "mpu6050.c" "AttitudeCtrl.c" "quaternion.c" "KalmanFilter.c"
                    INCLUDE_DIRS "." 
                    REQUIRES driver freertos)
