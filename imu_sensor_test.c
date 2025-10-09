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


#define M_PI (3.14159265358979323846)
#define DEG2RAD ((float)M_PI / 180.0f)

K_THREAD_STACK_DEFINE(t_stack_size, 2048);  /// Thread for logging 

static struct k_thread log_thread_data;  //thread

static struct sensor_trigger trig_mode, trig_mode_two; //trigger struct for both IMUs

static const struct device * mpu9250;  /// IMU instance for .overlay
static const struct device * mpu9250_2;

struct sensor_state {  /// struct state for each sensor, data for madgwick filter
    struct zsl_quat q;
    struct zsl_euler e;
    zsl_real_t acc_data[3];
    struct zsl_vec av;
    zsl_real_t gyr_data[3];
    struct zsl_vec gv;
    zsl_real_t mag_data[3];
    struct zsl_vec mv;
    uint32_t time;
};

static struct sensor_state s1_state;   // Declare them indivually because each has its own beta 
static struct sensor_state s2_state;

static zsl_real_t gyro_bias_1[3];
static zsl_real_t gyro_bias_2[3];

static void state_init(struct sensor_state * s){ //initialize each sensor state 
    s->q.r = 1.0;    // for calculation
    s->q.i = 0.0;
    s->q.j = 0.0;
    s->q.k = 0.0;

    s->av.sz = 3;      /// x, y, and z axis
    s->av.data = s->acc_data;

    s->gv.sz = 3;
    s->gv.data = s->gyr_data;

    s->mv.sz = 3;
    s->mv.data = s->mag_data;
}

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static struct zsl_fus_madg_cfg madg_cfg = {
    .beta = 0.174,
};

static struct zsl_fus_madg_cfg madg_cfg_2 = {
    .beta = 0.174,
};

static struct zsl_fus_drv madgwick_drv = {  /// madgwick filter code
    .init_handler = zsl_fus_madg_init,
    .feed_handler = zsl_fus_madg_feed,
    .error_handler = zsl_fus_madg_error,
    .config = &madg_cfg,
};

static struct zsl_fus_drv madgwick_drv_2 = {  /// madgwick filter code
    .init_handler = zsl_fus_madg_init,
    .feed_handler = zsl_fus_madg_feed,
    .error_handler = zsl_fus_madg_error,
    .config = &madg_cfg_2,
};

struct sensors_data {
    zsl_real_t acc1;
    zsl_real_t acc2;
    zsl_real_t acc3;
    zsl_real_t gyr1;
    zsl_real_t gyr2;
    zsl_real_t gyr3;
    zsl_real_t mag1;
    zsl_real_t mag2;
    zsl_real_t mag3;

};

struct log_data_t{
    uint8_t sensorId;
    zsl_real_t roll;
    zsl_real_t pitch;
    zsl_real_t yaw;
};

K_MSGQ_DEFINE(log_q, sizeof(struct log_data_t), 16, 4);  ///Create message queue

void logging_thread_funct(void * p1, void *p2, void *p3){ 
    struct log_data_t log_data;

    while (1){
        k_msgq_get(&log_q, &log_data, K_FOREVER);

         printf("Sensor# %d  Roll: %.2f  Pitch: %.2f Yaw: %.2f\n",
            log_data.sensorId,
               (double)log_data.roll,
               (double)log_data.pitch,
               (double)log_data.yaw);
    }

}

// void logging_thread_funct(void *p1, void *p2, void *p3) { 
//     struct log_data_t log_data;
//     struct log_data_t latest[2] = {0};
//     bool has_new[2] = {false, false};

//     while (1) {
//         k_msgq_get(&log_q, &log_data, K_FOREVER);

//         latest[log_data.sensorId] = log_data;  // store by sensorId
//         has_new[log_data.sensorId] = true;

//      
//         if (has_new[0] && has_new[1]) {
//             printf("Sensor1: R%.2f P%.2f Y%.2f | Sensor2: R%.2f P%.2f Y%.2f\n",
//                 (double)latest[0].roll, (double)latest[0].pitch, (double)latest[0].yaw,
//                 (double)latest[1].roll, (double)latest[1].pitch, (double)latest[1].yaw);

//            
//             has_new[0] = false;
//             has_new[1] = false;
//         }
//     }
// }

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

    data->acc1 = sensor_value_to_float(&acc[0]);
    data->acc2 = sensor_value_to_float(&acc[1]);
    data->acc3 = sensor_value_to_float(&acc[2]);
    data->gyr1 = sensor_value_to_float(&gyr[0]) * DEG2RAD;
    data->gyr2 = sensor_value_to_float(&gyr[1]) * DEG2RAD;
    data->gyr3 = sensor_value_to_float(&gyr[2]) * DEG2RAD;
    data->mag1 = sensor_value_to_float(&mag[0]);
    data->mag2 = sensor_value_to_float(&mag[1]);
    data->mag3 = sensor_value_to_float(&mag[2]);
    

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

    zsl_real_t sum_gyro[3] = {0};

    for(size_t i= 0; i < idx; i++){
        int ret = process_mpu9250(dev, &data);

        LOG_INF("%zu\n",i );

        if (ret != 0 ){
            LOG_ERR("Failed to set trigger mode in calibration");
            return;
        }
        acc_buf[i * 3 + 0] = data.acc1;
        acc_buf[i * 3 + 1] = data.acc2;
        acc_buf[i * 3 + 2] = data.acc3;

        gyr_buf[i * 3 + 0] = data.gyr1;
        gyr_buf[i * 3 + 1] = data.gyr2;
        gyr_buf[i * 3 + 2] = data.gyr3;

        sum_gyro[0] += data.gyr1;
        sum_gyro[1] += data.gyr2;
        sum_gyro[2] += data.gyr3;

        mag_buf[i * 3 + 0] = data.mag1;
        mag_buf[i * 3 + 1] = data.mag2;
        mag_buf[i * 3 + 2] = data.mag3;

    }
    if (dev == mpu9250) {
        gyro_bias_1[0] = sum_gyro[0]/idx;   
        gyro_bias_1[1] = sum_gyro[1]/idx;  
        gyro_bias_1[2] = sum_gyro[2]/idx;     
    } else if (dev == mpu9250_2) {
        gyro_bias_2[0] = sum_gyro[0]/idx;   
        gyro_bias_2[1] = sum_gyro[1]/idx;  
        gyro_bias_2[2] = sum_gyro[2]/idx;    
    }
     struct zsl_mtx acc = {  // Turn samples into matrix for all axis
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
                                                /// 100 Hz
            if (zsl_fus_cal_madg(&gyr, &acc, &mag, 100.0, NULL, &beta) == 0){
                 if (dev == mpu9250) {
                    madg_cfg.beta = beta;      // config for sen 1
                } else if (dev == mpu9250_2) {
                    madg_cfg_2.beta = beta;    // config for sen 2
                } else {
                    LOG_ERR("Unknown device for calibration");
                    return;
                }
                printf("Recalibrated beta = %f\n", (double) beta);
            } else{
                LOG_ERR("Calibraton failed \n");
                return;
            }

}

static void mpu_data_ready(const struct device *dev, const struct sensor_trigger * trig){

    struct sensors_data data;
    struct sensor_state * state;  // point to the state s1 or s2 that calls this function
    uint8_t sen_id;
    zsl_real_t local_biases[3];

    if (dev == mpu9250){
        sen_id = 1;
        state = &s1_state;

    }else if (dev == mpu9250_2){
        sen_id = 2;
        state = &s2_state;
    }else {
        LOG_ERR("Received invalid state");
        return;
    }

    int32_t now = k_uptime_get_32();  /// 10ms
    if (now - state->time < 10) {
        return;
    }

    state->time = now;
 
    int ret = process_mpu9250(dev, &data);   ///get raw data

    if (ret != 0){
        LOG_ERR("Trigger mode terminated; %d\n", ret);
        (void)sensor_trigger_set(dev, trig, NULL);
        return;
    }  

    local_biases[0] = (dev == mpu9250) ? gyro_bias_1[0] : gyro_bias_2[0];
    local_biases[1] = (dev == mpu9250) ? gyro_bias_1[1] : gyro_bias_2[1];
    local_biases[2] = (dev == mpu9250) ? gyro_bias_1[2] : gyro_bias_2[2];

    state->av.data[0] = data.acc1;
    state->av.data[1] = data.acc2;
    state->av.data[2] = data.acc3;

    state->gv.data[0] = data.gyr1 - local_biases[0];
    state->gv.data[1] = data.gyr2 - local_biases[1];
    state->gv.data[2] = data.gyr3 - local_biases[2];

    state->mv.data[0] = data.mag1;
    state->mv.data[1] = data.mag2;
    state->mv.data[2] = data.mag3;


    /// Apply the Madgwick Filter

    if (dev == mpu9250){
        madgwick_drv.feed_handler(&state->av, &state->mv, &state->gv, NULL, &state->q, madgwick_drv.config);

    }else if (dev == mpu9250_2){
        madgwick_drv_2.feed_handler(&state->av, &state->mv, &state->gv, NULL, &state->q, madgwick_drv_2.config);
    }
    
    zsl_quat_to_euler(&state->q, &state->e);

    state->e.x *= 180.0f / (float)ZSL_PI;
    state->e.y *= 180.0f / (float)ZSL_PI;
    state->e.z *= 180.0f / (float)ZSL_PI;

        struct log_data_t received_data = {  // save it to log struct for thread to print
            .sensorId = sen_id,
            .roll = state->e.x,
            .pitch = state->e.y,
            .yaw = state->e.z,
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

    state_init(&s1_state);
    state_init(&s2_state);

    mpu9250 = DEVICE_DT_GET(DT_NODELABEL(mpu_sensor));
    mpu9250_2 = DEVICE_DT_GET(DT_NODELABEL(mpu_sensor_two));

    if(!device_is_ready(mpu9250) || !device_is_ready(mpu9250_2)){
        LOG_ERR("The MPU is not ready");
    }

    thread_log_init();

    LOG_INF("Start calibrating the first IMU ...");
    madg_cal(mpu9250);
    LOG_INF("Calibration 1 completed.");
    k_sleep(K_MSEC(5));

    LOG_INF("Start calibrating the second IMU...");
    madg_cal(mpu9250_2);
    LOG_INF("Calibration 2 completed.");

    madgwick_drv.init_handler(100.0, madgwick_drv.config);

    trig_mode = (struct sensor_trigger){
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };

    trig_mode_two = (struct sensor_trigger){
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };

    if (sensor_trigger_set(mpu9250, &trig_mode, mpu_data_ready) < 0){
        LOG_ERR("Cannot figure the trigger mpu 1\n");
        return 0;
    }

      if (sensor_trigger_set(mpu9250_2, &trig_mode_two, mpu_data_ready) < 0){
        LOG_ERR("Cannot figure the trigger mpu 2\n");
        return 0;
    }

    LOG_INF("Configured the trigger mode...");

    while (1){
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
