#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <math.h>
#include <zsl/zsl.h>
#include <zsl/orientation/orientation.h>
#include <zsl/instrumentation.h>
#include <stdio.h>


K_THREAD_STACK_DEFINE(t_stack_size, 2048);

static struct k_thread log_thread_data;

static struct sensor_trigger trig_mode;


LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static struct zsl_fus_madg_cfg madg_cfg = {
    .beta = 0.1f,
};

// static struct zsl_fus_madg_cfg madg_cfg_two = {
//     .beta = 0.1f,
// };

static struct zsl_fus_drv madgwick_drv = {
    .init_handler = zsl_fus_madg_init,
    .feed_handler = zsl_fus_madg_feed,
    .error_handler = zsl_fus_madg_error,
    .config = &madg_cfg
};

// static struct zsl_fus_drv madgwick_drv_two = {
//     .init_handler = zsl_fus_madg_init,
//     .feed_handler = zsl_fus_madg_feed,
//     .error_handler = zsl_fus_madg_error,
//     .config = &madg_cfg_two
// };
struct sensors_data {
    double acc1;
     double acc2;
     double acc3;
    double gyr1;
    double gyr2;
     double gyr3;
     double mag1;
    double mag2;
    double mag3;

};

struct log_data_t{
    //uint8_t sensorId;
    double roll;
    double pitch;
    double yaw;
};

K_MSGQ_DEFINE(log_q, sizeof(struct log_data_t), 8, 4);

void logging_thread_funct(void * p1, void *p2, void *p3){
    struct log_data_t log_data;

    while (1){
        k_msgq_get(&log_q, &log_data, K_FOREVER);

         printf("Sensor# %d  Roll: %.2f  Pitch: %.2f Yaw: %.2f\n",
               // log_data.sensorId,
               log_data.roll,
               log_data.pitch,
               log_data.yaw);
    }
}

static int process_mpu9250(const struct device * dev, struct sensors_data *data){

    struct sensor_value temp;
    struct sensor_value acc[3], gyr[3], mag[3];
  

    int ret = sensor_sample_fetch(dev);

    if(ret == 0){
        ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
    }

     if(ret == 0){
        ret = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);
    }

     if(ret == 0){
        ret = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp);
    }

      if(ret == 0){
        ret = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag);
    }
    
    if (ret == 0){

    data->acc1 = sensor_value_to_double(&acc[0]);
    data->acc2 = sensor_value_to_double(&acc[1]);
    data->acc3 = sensor_value_to_double(&acc[2]);
    data->gyr1 = sensor_value_to_double(&gyr[0]);
    data->gyr2 = sensor_value_to_double(&gyr[1]);
    data->gyr3 = sensor_value_to_double(&gyr[2]);
    data->mag1 = sensor_value_to_double(&mag[0]);
    data->mag2 = sensor_value_to_double(&mag[1]);
    data->mag3 = sensor_value_to_double(&mag[2]);
    

    } else {
        LOG_ERR("Unable to fetch, failed: %d \n", ret);
    }

    return ret;
}


void madg_cal(const struct device *dev){  /// calibration

    struct sensors_data data;

    //Temp buffer for stationary calibration
    static zsl_real_t acc_buf[60 * 3];
    static zsl_real_t gyr_buf[60 * 3];
    static zsl_real_t mag_buf[60 * 3];
    size_t idx = 60;

    for(size_t i= 0; i < idx; i++){
        int ret = process_mpu9250(dev, &data);

        LOG_INF("%zu\n",i );

        if (ret != 0 ){
            LOG_ERR("Failed to set trigger mode in calibration");
            return;
        }
        acc_buf[i * 3 + 0] = (zsl_real_t)data.acc1;
        acc_buf[i * 3 + 1] = (zsl_real_t)data.acc2;
        acc_buf[i * 3 + 2] = (zsl_real_t)data.acc3;

        gyr_buf[i * 3 + 0] = (zsl_real_t)data.gyr1;
        gyr_buf[i * 3 + 1] = (zsl_real_t)data.gyr2;
        gyr_buf[i * 3 + 2] = (zsl_real_t)data.gyr3;

        mag_buf[i * 3 + 0] = (zsl_real_t)data.mag1;
        mag_buf[i * 3 + 1] = (zsl_real_t)data.mag2;
        mag_buf[i * 3 + 2] = (zsl_real_t)data.mag3;

    }
     struct zsl_mtx acc = {  // Turn samples into matrix for all  axis
                .sz_rows = 60,
                .sz_cols = 3,
                .data = acc_buf,
            };

             struct zsl_mtx gyr = {
                .sz_rows = 60,
                .sz_cols = 3,
                .data = gyr_buf,
            };

             struct zsl_mtx mag = {
                .sz_rows = 60,
                .sz_cols = 3,
                .data = mag_buf,
            };

            zsl_real_t beta;

            if (zsl_fus_cal_madg(&gyr, &acc, &mag, 100.0, NULL, &beta) == 0){
                madg_cfg.beta = beta;
                printf("Recalibrated beta = %f\n", (double) beta);
            } else{
                LOG_ERR("Calibraton failed \n");
                return;
            }


}


static void mpu_data_ready(const struct device *dev, const struct sensor_trigger * trig){

    static struct sensors_data data;

    static struct zsl_quat q = { .r = 1.0, .i = 0.0f, .j = 0.0f, .k = 0.0f };
    static struct zsl_euler e ={ 0 };

    static ZSL_VECTOR_DEF(av, 3);
    static ZSL_VECTOR_DEF(gv, 3);
    static ZSL_VECTOR_DEF(mv, 3);


    static uint32_t last_ms = 0;
    int32_t now = k_uptime_get_32();

    if (now - last_ms < 10){
        return;
    }

    last_ms = now;

    int ret = process_mpu9250(dev, &data);

    if (ret != 0){
        LOG_ERR("Trigger mode terminated; %d\n", ret);
        (void)sensor_trigger_set(dev, trig, NULL);
        return;
    }

    av.data[0] = data.acc1;
    av.data[1] = data.acc2;
    av.data[2] = data.acc3;

    gv.data[0] = data.gyr1;
    gv.data[1] = data.gyr2;
    gv.data[2] = data.gyr3;

    mv.data[0] = data.mag1;
    mv.data[1] = data.mag2;
    mv.data[2] = data.mag3;

    madgwick_drv.feed_handler(&av, &mv, &gv, NULL, &q, madgwick_drv.config);

        
        zsl_quat_to_euler(&q, &e);

       e.x *= 180.0f / (float)ZSL_PI;
        e.y *= 180.0f / (float)ZSL_PI;
        e.z *= 180.0f / (float)ZSL_PI;

        struct log_data_t received_data = {
            .roll = e.x,
            .pitch = e.y,
            .yaw = e.z,
        };

        k_msgq_put(&log_q, &received_data, K_NO_WAIT);

     //printf("Yaw: %.2f deg \n", (double)MadgwickGetYawDeg());
}

void thread_log_init(void){
     k_tid_t logging_tid = k_thread_create(&log_thread_data, t_stack_size,
                                        K_THREAD_STACK_SIZEOF(t_stack_size),
                                        logging_thread_funct,
                                        NULL, NULL, NULL,
                                        6, 
                                        0, K_NO_WAIT);
}

int main(void){

    const struct device * const mpu9250 = DEVICE_DT_GET(DT_NODELABEL(mpu_sensor));

    if(!device_is_ready(mpu9250)){
        LOG_ERR("The MPU: %s is not ready\n", mpu9250->name);
    }

    thread_log_init();
    LOG_INF("Start calibrating...");
    madg_cal(mpu9250);
    LOG_INF("Calibration completed.");

    madgwick_drv.init_handler(100.0, madgwick_drv.config);

    trig_mode = (struct sensor_trigger){
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };

    if (sensor_trigger_set(mpu9250, &trig_mode, mpu_data_ready) < 0){
        LOG_ERR("Cannot figure the trigger\n");
        return 0;
    }

    LOG_INF("Configured the trigger mode...");

    while (1){
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
