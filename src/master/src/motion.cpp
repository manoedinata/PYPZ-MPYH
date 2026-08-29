#include "master/master.hpp"

float Master::pythagoras(float x1, float y1, float x2, float y2)
{
    return sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

void Master::manual_motion(float vx, float vy, float wz)
{
    static float vx_buffer = 0; // Throttle velocity
    static float wz_buffer = 0; // Steering angle

    (void)vy;

    /**
     * Menghitung kecepatan mobil
     * Braking system aktif ketika vx < 0, sisanya kontrol kecepatan
     */

    /**
     * Ketika brake
     */
    static uint8_t state_control = 0;
    static uint8_t prev_state_control = 0;
    if (vx < 0)
    {
        state_control = 0;
        if (prev_state_control != 0)
            actuation_ax = 0;

        actuation_ax += -profile_max_braking_jerk * 0.5 * dt * dt;
        if (actuation_ax < -profile_max_braking_acceleration)
            actuation_ax = -profile_max_braking_acceleration;
        vx_buffer += actuation_ax * dt;
        if (vx_buffer < vx)
            vx_buffer = vx;
    }
    /**
     * Accelerate setelah braking, segera lepas pedal brake
     */
    else if (vx > 0 && vx_buffer < 0)
    {
        state_control = 1;
        if (prev_state_control != 1)
            actuation_ax = 0;

        actuation_ax += profile_max_braking_jerk * 0.5 * dt * dt;
        if (actuation_ax > profile_max_braking_acceleration)
            actuation_ax = profile_max_braking_acceleration;
        vx_buffer += actuation_ax * dt;
        if (vx_buffer > vx)
            vx_buffer = vx;
    }
    /**
     * Normal acceleration
     */
    else if (vx > vx_buffer)
    {
        state_control = 2;
        if (prev_state_control != 2)
            actuation_ax = 0;

        actuation_ax += profile_max_accelerate_jerk * 0.5 * dt * dt;
        if (actuation_ax > profile_max_acceleration)
            actuation_ax = profile_max_acceleration;
        vx_buffer += actuation_ax * dt;
        if (vx_buffer > vx)
            vx_buffer = vx;
    }
    /**
     * Normal deceleration
     */
    else if (vx < vx_buffer)
    {
        state_control = 3;
        if (prev_state_control != 3)
            actuation_ax = 0;

        actuation_ax -= profile_max_decelerate_jerk * 0.5 * dt * dt;
        if (actuation_ax < -profile_max_decceleration)
            actuation_ax = -profile_max_decceleration;
        vx_buffer += actuation_ax * dt;
        if (vx_buffer < vx)
            vx_buffer = vx;
    }
    prev_state_control = state_control;

    /**
     * Menghitung kecepatan steer berdasarkan kecepatan mobil
     * Semakin cepat mobil, semakin lambat perputaran steer
     */
    // static const float min_velocity = 2.0 / 3.6;              // 2 km/h
    // static const float max_velocity = 7.0 / 3.6;              // 7 km/h
    // static const float min_steering_rate = 15 * M_PI / 180.0; // 7 deg/s
    // static const float max_steering_rate = 36 * M_PI / 180.0; // 21 deg/s
    // static const float gradient_steering_rate = (min_steering_rate - max_steering_rate) / (max_velocity - min_velocity);

    // float steering_rate = fmaxf(min_steering_rate,
    //                             fminf(max_steering_rate,
    //                                   gradient_steering_rate * (fb_final_vel_dxdydo[0] - min_velocity) + max_steering_rate));

    float steering_rate = 99.0;

    if (wz > wz_buffer)
    {
        wz_buffer += steering_rate * dt;
        if (wz_buffer > wz)
            wz_buffer = wz;
    }
    else if (wz < wz_buffer)
    {
        wz_buffer -= steering_rate * dt;
        if (wz_buffer < wz)
            wz_buffer = wz;
    }

    if (vx_buffer > profile_max_velocity)
        vx_buffer = profile_max_velocity;
    else if (vx_buffer < -profile_max_velocity)
        vx_buffer = -profile_max_velocity;

    if (wz_buffer > profile_max_steering_rad)
        wz_buffer = profile_max_steering_rad;
    else if (wz_buffer < -profile_max_steering_rad)
        wz_buffer = -profile_max_steering_rad;

    actuation_vx = vx_buffer;
    actuation_ay = 0;
    actuation_wz = wz_buffer;

    // target_velocity = fmaxf(0, vx_buffer);

    // logger.info("%.2f %.2f || %.2f %.2f || %.2f %.2f || %.2f %.2f", vx, wz, actuation_ax, steering_rate, vx_buffer, wz_buffer, actuation_vx, actuation_wz);
}

int8_t Master::move_forward_distance(float distance_target, float *pvelocity, float start_x, float start_y, float target_distance)
{
    static float error_prev = 0.0;
    static float error_integral = 0.0;
    static bool motion_done = false;
    static float dt = 0.05;

    // Ambil posisi sekarang (misalnya dari odometri atau estimasi posisi)
    float current_position = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], start_x, start_y);
    float error = distance_target - current_position;

    // Parameter PID
    const float Kp = 3.2;
    const float Ki = 0.1;
    const float Kd = 0.0;

    // PID control
    error_integral += error * dt;
    float error_derivative = (error - error_prev) / dt;
    error_prev = error;

    float control_vx = Kp * error + Ki * error_integral + Kd * error_derivative;

    // Batas kecepatan maksimum
    const float max_vx = 0.65; // m/s
    if (control_vx > max_vx)
        control_vx = max_vx;
    else if (control_vx < -max_vx)
        control_vx = -max_vx;

    // Deteksi kondisi selesai

    logger.info("Current Position: %.2f, Target Distance: %.2f, Error: %.2f, Control Vx: %.2f",
                current_position, distance_target, error, control_vx);

    *pvelocity = control_vx;

    if (fabs(error) < target_distance)
    {
        control_vx = 0.0;
        motion_done = true;
        return 1;
    }
    else
    {
        return 0;
    }
    // manual_motion(control_vx, 0.0, 0.0); // panggil fungsi throttle + steering kamu
}

void Master::wp2velocity_steering(float lookahead_distance, float *pvelocity, float *psteering, bool is_loop)
{
    if (waypoints.size() == 0)
    {
        *pvelocity = 0;
        *psteering = 0;
        return;
    }

    static uint32_t index_sekarang = 0;
    static uint32_t index_lookahead = 0;
    float obs_scan_r = 2;
    static const float threshold_error_sangat_besar = 0.15;
    static const float thresholad_error_kecil = 0.04;
    static const int16_t error_kecil_window_index_search = 10;
    static const float offset_fb_velocity = 2;

    /* Mencari index sekarang pada waypoints */
    float error_index_sekarang = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints[index_sekarang].x, waypoints[index_sekarang].y);
    if (error_index_sekarang > threshold_error_sangat_besar)
    {
        float min_error = FLT_MAX;
        uint32_t index_terdekat = 0;
        for (size_t i = 0; i < waypoints.size(); i++)
        {
            float error = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints[i].x, waypoints[i].y);
            if (error < min_error)
            {
                min_error = error;
                index_terdekat = i;
            }
        }
        index_sekarang = index_terdekat;
    }
    else if (error_index_sekarang > thresholad_error_kecil)
    {
        float min_error = FLT_MAX;
        uint32_t index_terdekat = 0;
        for (size_t i = index_sekarang - error_kecil_window_index_search; i < index_sekarang + error_kecil_window_index_search * 2; i++)
        {
            float error = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints[i].x, waypoints[i].y);
            if (error < min_error)
            {
                min_error = error;
                index_terdekat = i;
            }
        }
        index_sekarang = index_terdekat;
    }

    /* Menentukan efek terminal */
    static float max_velocity = profile_max_velocity;
    for (size_t i = 0; i < terminals.terminals.size(); i++)
    {
        float jarak_robot_terminal = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], terminals.terminals[i].target_pose_x, terminals.terminals[i].target_pose_y);
        if (jarak_robot_terminal < terminals.terminals[i].radius_area)
        {
            lookahead_distance = terminals.terminals[i].target_lookahead_distance;
            obs_scan_r = terminals.terminals[i].obs_scan_r;
            max_velocity = terminals.terminals[i].target_max_velocity_x;
            break;
        }
    }

    /* Mencari waypoint sesuai lookahead_distance */
    index_lookahead = index_sekarang;
    if (!is_loop)
    {
        for (size_t i = index_sekarang; i < waypoints.size(); i++)
        {
            float error = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints[i].x, waypoints[i].y);
            if (error > lookahead_distance)
            {
                index_lookahead = i;
                break;
            }
        }
    }
    else
    {
        for (size_t i = index_sekarang; i < waypoints.size() * 2; i++)
        {
            size_t index_used = i;

            if (i >= waypoints.size())
                index_used -= waypoints.size();

            float error = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints[index_used].x, waypoints[index_used].y);
            if (error > lookahead_distance)
            {
                index_lookahead = index_used;
                break;
            }
        }
    }

    /* Jika sudah mencapai waypoint terakhir */
    if (index_lookahead == waypoints.size() - 1)
    {
        if (is_loop)
        {
            index_lookahead = 0;
        }
        else
        {
            *pvelocity = 0;
            *psteering = 0;
            return;
        }
    }

    /* Safety tambahan */
    if (index_lookahead == index_sekarang)
    {
        *pvelocity = 0;
        *psteering = 0;
        return;
    }

    /* Menghitung target velocity */
    float dx = waypoints[index_lookahead].x - fb_final_pose_xyo[0];
    float dy = waypoints[index_lookahead].y - fb_final_pose_xyo[1];
    float target_velocity = waypoints[index_lookahead].fb_velocity + offset_fb_velocity;

    if (target_velocity > FLT_EPSILON && target_velocity < 0.48)
        target_velocity = fmaxf(0.48, target_velocity);

    if (target_velocity > max_velocity)
        target_velocity = max_velocity;

    /* Menghitung target steering angle */
    float direction = atan2(dy, dx) - fb_final_pose_xyo[2];
    float target_steering_angle = atan2(2 * wheelbase * sinf(direction), lookahead_distance);
    while (target_steering_angle > M_PI)
        target_steering_angle -= 2 * M_PI;
    while (target_steering_angle < -M_PI)
        target_steering_angle += 2 * M_PI;

    // logger.info("%.2f %.2f", target_velocity, target_steering_angle);

    /* Menghitung obstacle */
    // if (enable_obs_detection)
    // {
    //     std_msgs::msg::Float32 msg_obs_find;
    //     msg_obs_find.data = local_obstacle_influence(obs_scan_r, 8.0);
    //     pub_obs_find->publish(msg_obs_find);
    //     if (msg_obs_find.data > 0.05)
    //     {
    //         target_velocity = fmaxf(-profile_max_braking, target_velocity - msg_obs_find.data);

    //         if (target_velocity > FLT_EPSILON && target_velocity < 0.48)
    //             target_velocity = -1;

    //         if (target_velocity > max_velocity)
    //             target_velocity = max_velocity;
    //     }
    // }

    if (target_steering_angle > profile_max_steering_rad)
        target_steering_angle = profile_max_steering_rad;
    else if (target_steering_angle < -profile_max_steering_rad)
        target_steering_angle = -profile_max_steering_rad;

    *pvelocity = target_velocity;
    *psteering = target_steering_angle;
}

void Master::wp2velocity_steering_race(float lookahead_distance, float *pvelocity, float *psteering, bool is_loop)
{
    if (waypoints.size() == 0)
    {
        *pvelocity = 0;
        *psteering = 0;
        return;
    }

    static uint32_t index_sekarang = 0;
    static uint32_t index_lookahead = 0;
    float obs_scan_r = 2;
    static const float threshold_error_sangat_besar = 0.15;
    static const float thresholad_error_kecil = 0.04;
    static const int16_t error_kecil_window_index_search = 10;
    static const float offset_fb_velocity = 2;
    static int8_t prev_selected_lane = 0;

    if (selected_lane == 0)
        waypoints_race_selected = waypoints;
    else if (selected_lane == -1)
        waypoints_race_selected = waypoints_race_kiri;
    else if (selected_lane == 1)
        waypoints_race_selected = waypoints_race_kanan;

    // waypoints_race_selected = waypoints_race_kanan;

    // logger.info("============== wp2velocity_steering_race ===============");

    // logger.info("%s || size: %d", selected_lane == 0 ? "tengah" : selected_lane == 1 ? "kanan"
    //                                                                                  : "kiri",
    //             waypoints_race_selected.size());

    if (prev_selected_lane != selected_lane)
    {
        // index_sekarang = 0;
    }

    float min_error = FLT_MAX;
    uint32_t index_terdekat = 0;
    for (size_t i = 0; i < waypoints_race_selected.size(); i++)
    {
        float error = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints_race_selected[i].x, waypoints_race_selected[i].y);
        if (error < min_error)
        {
            min_error = error;
            index_terdekat = i;
        }
    }
    index_sekarang = index_terdekat;

    /* Mencari index sekarang pada waypoints */
    // float error_index_sekarang = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints_race_selected[index_sekarang].x, waypoints_race_selected[index_sekarang].y);
    // if (error_index_sekarang > threshold_error_sangat_besar)
    // {
    //     float min_error = FLT_MAX;
    //     uint32_t index_terdekat = 0;
    //     for (size_t i = 0; i < waypoints_race_selected.size(); i++)
    //     {
    //         float error = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints_race_selected[i].x, waypoints_race_selected[i].y);
    //         if (error < min_error)
    //         {
    //             min_error = error;
    //             index_terdekat = i;
    //         }
    //     }
    //     index_sekarang = index_terdekat;
    // }
    // else if (error_index_sekarang > thresholad_error_kecil)
    // {
    //     float min_error = FLT_MAX;
    //     uint32_t index_terdekat = 0;
    //     for (size_t i = index_sekarang - error_kecil_window_index_search; i < index_sekarang + error_kecil_window_index_search * 2; i++)
    //     {
    //         float error = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints_race_selected[i].x, waypoints_race_selected[i].y);
    //         if (error < min_error)
    //         {
    //             min_error = error;
    //             index_terdekat = i;
    //         }
    //     }
    //     index_sekarang = index_terdekat;
    // }

    /* Menentukan efek terminal */
    static float max_velocity = profile_max_velocity;
    for (size_t i = 0; i < terminals.terminals.size(); i++)
    {
        float jarak_robot_terminal = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], terminals.terminals[i].target_pose_x, terminals.terminals[i].target_pose_y);
        if (jarak_robot_terminal < terminals.terminals[i].radius_area)
        {
            lookahead_distance = terminals.terminals[i].target_lookahead_distance;
            obs_scan_r = terminals.terminals[i].obs_scan_r;
            max_velocity = terminals.terminals[i].target_max_velocity_x;
            break;
        }
    }

    /* Mencari waypoint sesuai lookahead_distance */
    index_lookahead = index_sekarang;
    if (!is_loop)
    {
        for (size_t i = index_sekarang; i < waypoints_race_selected.size(); i++)
        {
            float error = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints_race_selected[i].x, waypoints_race_selected[i].y);
            if (error > lookahead_distance)
            {
                index_lookahead = i;
                break;
            }
        }
    }
    else
    {
        for (size_t i = index_sekarang; i < waypoints_race_selected.size() * 2; i++)
        {
            size_t index_used = i;

            if (i >= waypoints_race_selected.size())
                index_used -= waypoints_race_selected.size();

            float error = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints_race_selected[index_used].x, waypoints_race_selected[index_used].y);
            if (error > lookahead_distance)
            {
                index_lookahead = index_used;
                break;
            }
        }
    }

    prev_selected_lane = selected_lane;

    /* Jika sudah mencapai waypoint terakhir */
    if (index_lookahead == waypoints_race_selected.size() - 1)
    {
        if (is_loop)
        {
            index_lookahead = 0;
        }
        else
        {
            *pvelocity = 0;
            *psteering = 0;
            return;
        }
    }

    /* Safety tambahan */
    if (index_lookahead == index_sekarang)
    {
        *pvelocity = 0;
        *psteering = 0;
        return;
    }

    /* Menghitung target velocity */
    float dx = waypoints_race_selected[index_lookahead].x - fb_final_pose_xyo[0];
    float dy = waypoints_race_selected[index_lookahead].y - fb_final_pose_xyo[1];
    float target_velocity = waypoints_race_selected[index_lookahead].fb_velocity + offset_fb_velocity;

    if (target_velocity > FLT_EPSILON && target_velocity < 0.48)
        target_velocity = fmaxf(0.48, target_velocity);

    if (target_velocity > max_velocity)
        target_velocity = max_velocity;

    /* Menghitung target steering angle */
    float direction = atan2(dy, dx) - fb_final_pose_xyo[2];
    float target_steering_angle = atan2(2 * wheelbase * sinf(direction), lookahead_distance);
    while (target_steering_angle > M_PI)
        target_steering_angle -= 2 * M_PI;
    while (target_steering_angle < -M_PI)
        target_steering_angle += 2 * M_PI;

    logger.info("%.2f %.2f | %.2f", target_velocity, target_steering_angle, lookahead_distance);

    /* Menghitung obstacle */
    // if (enable_obs_detection)
    // {
    //     std_msgs::msg::Float32 msg_obs_find;
    //     msg_obs_find.data = local_obstacle_influence(obs_scan_r, 8.0);
    //     pub_obs_find->publish(msg_obs_find);
    //     if (msg_obs_find.data > 0.05)
    //     {
    //         target_velocity = fmaxf(-profile_max_braking, target_velocity - msg_obs_find.data);

    //         if (target_velocity > FLT_EPSILON && target_velocity < 0.48)
    //             target_velocity = -1;

    //         if (target_velocity > max_velocity)
    //             target_velocity = max_velocity;
    //     }
    // }

    if (target_steering_angle > profile_max_steering_rad)
        target_steering_angle = profile_max_steering_rad;
    else if (target_steering_angle < -profile_max_steering_rad)
        target_steering_angle = -profile_max_steering_rad;

    *pvelocity = target_velocity;
    *psteering = target_steering_angle;
}

void Master::wp2velocity_steering_urban(float lookahead_distance, float *pvelocity, float *psteering, int *counter_diam, point_t arah_belok, int32_t sign_status, bool is_loop)
{
    if (waypoints.size() == 0)
    {
        *pvelocity = 0;
        *psteering = 0;
        *counter_diam = 0;
        return;
    }

    static uint32_t index_sekarang = 0;
    static uint32_t index_lookahead = 0;
    float obs_scan_r = 2;
    static const float threshold_error_sangat_besar = 0.15;
    static const float thresholad_error_kecil = 0.04;
    static const int16_t error_kecil_window_index_search = 10;
    static const float offset_fb_velocity = 2;
    int sedang_diam = 0;

    /* Mencari index sekarang pada waypoints */
    float error_index_sekarang = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints[index_sekarang].x, waypoints[index_sekarang].y);
    if (error_index_sekarang > threshold_error_sangat_besar)
    {
        float min_error = FLT_MAX;
        uint32_t index_terdekat = 0;
        for (size_t i = 0; i < waypoints.size(); i++)
        {
            float error = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints[i].x, waypoints[i].y);
            if (error < min_error)
            {
                min_error = error;
                index_terdekat = i;
            }
        }
        index_sekarang = index_terdekat;
    }
    else if (error_index_sekarang > thresholad_error_kecil)
    {
        float min_error = FLT_MAX;
        uint32_t index_terdekat = 0;
        for (size_t i = index_sekarang - error_kecil_window_index_search; i < index_sekarang + error_kecil_window_index_search * 2; i++)
        {
            float error = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints[i].x, waypoints[i].y);
            if (error < min_error)
            {
                min_error = error;
                index_terdekat = i;
            }
        }
        index_sekarang = index_terdekat;
    }

    /* Menentukan efek terminal */
    static float max_velocity = profile_max_velocity;
    for (size_t i = 0; i < terminals.terminals.size(); i++)
    {
        float jarak_robot_terminal = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], terminals.terminals[i].target_pose_x, terminals.terminals[i].target_pose_y);
        if (jarak_robot_terminal < terminals.terminals[i].radius_area)
        {
            if (terminals.terminals[i].type == TERMINAL_TYPE_STOP1)
            {
                if (sign_status != -1)
                {
                    lookahead_distance = terminals.terminals[i].target_lookahead_distance;
                    obs_scan_r = terminals.terminals[i].obs_scan_r;
                    max_velocity = terminals.terminals[i].target_max_velocity_x;

                    sedang_diam = 1;
                    break;
                }
            }
            else
            {
                lookahead_distance = terminals.terminals[i].target_lookahead_distance;
                obs_scan_r = terminals.terminals[i].obs_scan_r;
                max_velocity = terminals.terminals[i].target_max_velocity_x;
                break;
            }
        }
    }

    /* Mencari waypoint sesuai lookahead_distance */
    index_lookahead = index_sekarang;
    if (!is_loop)
    {
        float min_dist = FLT_MAX;
        for (size_t i = index_sekarang; i < waypoints.size(); i++)
        {
            float error = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints[i].x, waypoints[i].y);
            if (error > lookahead_distance)
            {
                index_lookahead = i;
                break;
            }
        }
    }
    else
    {
        float min_dist = FLT_MAX;

        for (size_t i = index_sekarang; i < waypoints.size() * 2; i++)
        {
            size_t index_used = i;

            if (i >= waypoints.size())
                index_used -= waypoints.size();

            if ((sign_status == 2 || sign_status == 3))
            {

                float error = pythagoras(arah_belok.x, arah_belok.y, waypoints[index_used].x, waypoints[index_used].y);
                if (error < min_dist)
                {
                    index_lookahead = index_used;
                    min_dist = error;
                }
            }
            else
            {
                float error = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], waypoints[index_used].x, waypoints[index_used].y);
                if (error > lookahead_distance)
                {
                    index_lookahead = index_used;
                    break;
                }
            }
        }
    }

    logger.info("pose: {%.3f, %.3f, %.3f}, target pos: {%.3f %.3f} || %.2f | %d -> %d", fb_final_pose_xyo[0], fb_final_pose_xyo[1], fb_final_pose_xyo[2], arah_belok.x, arah_belok.y, pythagoras(arah_belok.x, arah_belok.y, waypoints[index_lookahead].x, waypoints[index_lookahead].y), index_lookahead, index_sekarang);

    /* Jika sudah mencapai waypoint terakhir */
    if (index_lookahead == waypoints.size() - 1)
    {
        if (is_loop)
        {
            index_lookahead = 0;
        }
        else
        {
            *pvelocity = 0;
            *psteering = 0;
            *counter_diam = sedang_diam;
            return;
        }
    }

    /* Safety tambahan */
    if (index_lookahead == index_sekarang)
    {
        *pvelocity = 0;
        *psteering = 0;
        *counter_diam = sedang_diam;

        return;
    }

    /* Menghitung target velocity */
    float dx = waypoints[index_lookahead].x - fb_final_pose_xyo[0];
    float dy = waypoints[index_lookahead].y - fb_final_pose_xyo[1];
    float target_velocity = waypoints[index_lookahead].fb_velocity + offset_fb_velocity;

    if (target_velocity > FLT_EPSILON && target_velocity < 0.48)
        target_velocity = fmaxf(0.48, target_velocity);

    if (target_velocity > max_velocity)
        target_velocity = max_velocity;

    /* Menghitung target steering angle */
    float direction = atan2(dy, dx) - fb_final_pose_xyo[2];
    float target_steering_angle = atan2(2 * wheelbase * sinf(direction), lookahead_distance);
    while (target_steering_angle > M_PI)
        target_steering_angle -= 2 * M_PI;
    while (target_steering_angle < -M_PI)
        target_steering_angle += 2 * M_PI;

    logger.info("============= targer wp: %.2f %.2f || arah wp: %.2f", waypoints[index_lookahead].x, waypoints[index_lookahead].y, waypoints[index_lookahead].arah);

    if (target_steering_angle > profile_max_steering_rad)
        target_steering_angle = profile_max_steering_rad;
    else if (target_steering_angle < -profile_max_steering_rad)
        target_steering_angle = -profile_max_steering_rad;

    *pvelocity = target_velocity;
    *psteering = target_steering_angle;
    *counter_diam = sedang_diam;
}

void Master::wp2velocity_steering_urban_coba(float lookahead_distance, float *pvelocity, float *psteering, bool diam, bool *masuk_terminal_diam, bool is_loop)
{
}

void Master::follow_waypoints_gas_manual(float vx, float vy, float wz, float lookahead_distance, bool is_loop)
{
    (void)vy;
    (void)wz;

    float target_velocity = 0;
    float target_steering_angle = 0;
    wp2velocity_steering(lookahead_distance, &target_velocity, &target_steering_angle, is_loop);

    manual_motion(vx, 0, target_steering_angle);
}

void Master::follow_waypoints(float vx, float vy, float wz, float lookahead_distance, bool is_loop)
{
    (void)vx;
    (void)vy;
    (void)wz;

    float target_velocity = 0;
    float target_steering_angle = 0;
    wp2velocity_steering(lookahead_distance, &target_velocity, &target_steering_angle, is_loop);

    manual_motion(target_velocity, 0, target_steering_angle);
}

void Master::cnn_move(float vx, float vy, float wz, float profile_max_velocity, float target_steering_cnn)
{

    combine_road_obstacle_pcl();

    logger.info("CNN MOVE: %.2f %.2f || %.2f %.2f || %.2f %.2f", fb_final_pose_xyo[0], fb_final_pose_xyo[1], fb_final_vel_dxdydo[0], fb_final_vel_dxdydo[1], vx, wz);

    // If the nearest obstacle is too close, stop or adjust the steering
    // Otherwise, follow the waypoints normally with the CNN steering angle
    static float target_velocity_final = 0;
    static float target_steering_final = 0;
    static float prev_steering_obs = 0;
    static int8_t prev_ada_obs = 0;
    static point_t point_menghindar = {0, 0};
    static point_t global_obs_pos = {0, 0};

    static int cntr_obs_hilang = 0;

    static int fsm_state = 0;

    float dist = pythagoras(0, 0, obstacle_centroid.x(), obstacle_centroid.y());

    target_steering_final = target_steering_cnn;
    target_velocity_final = 0.15;

    logger.info("ada_obs: %d centroid: (%.2f, %.2f) || %.2f %.2f", ada_obs, obstacle_centroid.x(), obstacle_centroid.y(), fb_final_pose_xyo[0], fb_final_pose_xyo[1]);

    if (ada_obs == 0 && cntr_obs_hilang < 70)
    {
        cntr_obs_hilang++;

        if (cntr_obs_hilang > 20)
            fsm_state = 0;
    }
    else
    {
        cntr_obs_hilang = 0; // Reset counter if obstacle is detected
    }

    if (obstacle_centroid.x() < 1.4)
        target_velocity_final = kecepatan_default_menghindar;

    // jika ada obstacle
    if ((obstacle_centroid.x() < 1 && ada_obs == 1) && (fsm_state != 1 && fsm_state != 2))
    {
        fsm_state = 1; // Set state ke 1 jika ada obstacle
        global_obs_pos = {obstacle_centroid.x() + fb_final_pose_xyo[0], obstacle_centroid.y() + fb_final_pose_xyo[1]};
    }
    if ((obstacle_centroid.x() < 1 && ada_obs == 2) && (fsm_state != 1 && fsm_state != 2))
    {
        fsm_state = 1; // Set state ke 1 jika ada obstacle
        global_obs_pos = {obstacle_centroid.x() + fb_final_pose_xyo[0], obstacle_centroid.y() + fb_final_pose_xyo[1]};
    }

    // if (ada_obs == 0 && prev_ada_obs == 0 && fsm_state != 0)
    // {
    //     fsm_state = 0; // Reset state jika tidak ada obstacle
    //     target_velocity_final = vx;
    //     target_steering_final = wz;
    // }

    if (fsm_state == 0)
    {
        if (fabs(target_steering_final) > 0.05)
        {
            target_velocity_final *= 0.7;
            if (target_velocity_final < 0.8)
                target_velocity_final = 0.8; // Minimum speed to avoid stalling
        }
        if (fabs(target_steering_final) > 0.1)
        {
            target_velocity_final *= 0.7;
            if (target_velocity_final < 0.7)
                target_velocity_final = 0.7; // Minimum speed to avoid stalling
        }
        if (fabs(target_steering_final) > 0.13)
        {
            target_velocity_final *= 0.5;
            if (target_velocity_final < 0.6)
                target_velocity_final = 0.6; // Minimum speed to avoid stalling
        }

        // check if the robot is turning left or right

        // Apply the final target velocity and steering angle
    }
    else if (fsm_state == 1)
    {
        target_velocity_final = kecepatan_default_menghindar;
        // arahkan steering ke target point menghindar
        logger.info("FSM STATE 1: %.2f %.2f || %.2f || dist: %.2f", point_menghindar.x, point_menghindar.y, atan2(point_menghindar.y - fb_final_pose_xyo[1], point_menghindar.x - fb_final_pose_xyo[0]), pythagoras(global_obs_pos.x, global_obs_pos.y, fb_final_pose_xyo[0], fb_final_pose_xyo[1]));

        // buat supaya steering mengikuti posisi obstacle
        if (obstacle_centroid.x() > 0.5)
            point_menghindar = {obstacle_centroid.x(), obstacle_centroid.y() + offset_jarak_hindar}; // Maju ke kiri

        target_steering_final = atan2(point_menghindar.y, point_menghindar.x);
        // target_steering_final = 0.15;

        if (cntr_obs_hilang > lama_waktu_menghindar)
            fsm_state = 0; // Set state ke 2 jika tidak ada obstacle
    }
    else if (fsm_state == 2)
    {
        target_velocity_final = kecepatan_default_menghindar;

        logger.info("FSM STATE 2: %.2f %.2f || %.2f || dist: %.2f", point_menghindar.x, point_menghindar.y, atan2(point_menghindar.y - fb_final_pose_xyo[1], point_menghindar.x - fb_final_pose_xyo[0]), pythagoras(global_obs_pos.x, global_obs_pos.y, fb_final_pose_xyo[0], fb_final_pose_xyo[1]));

        // buat supaya steering mengikuti posisi obstacle
        point_menghindar = {obstacle_centroid.x(), obstacle_centroid.y() + offset_jarak_hindar}; // Maju ke kanan

        target_steering_final = atan2(point_menghindar.y, point_menghindar.x);
        // target_steering_final = 0.15;

        if (cntr_obs_hilang > lama_waktu_menghindar)
            fsm_state = 0; // Set state ke 2 jika tidak ada obstacle
    }
    else if (fsm_state == 3)
    {
        static float cntr_merubah_jalan = 0;

        if (cntr_merubah_jalan++ > 20)
        {
            cntr_merubah_jalan = 0;
            fsm_state = 0;
        }

        target_steering_final = target_steering_cnn;
    }

    target_velocity_final = fminf(vx, target_velocity_final);

    logger.info("vel: %.2f, steer: %.2f || %d [%d %d]", target_velocity_final, target_steering_final, fsm_state, ada_obs, prev_ada_obs);
    manual_motion(target_velocity_final, 0, target_steering_final);
}

void Master::cnn_move2(float vx, float vy, float wz, float profile_max_velocity, float target_steering_cnn)
{
}

void Master::race_move(float vx, float vy, float wz)
{
}

void Master::urban_move2(float vx, float vy, float wz, int8_t oto)
{
    static MachineState urban_fsm;
    urban_fsm.reentry(999, 0.4);

    double time_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    static float target_steering = 0;
    static float target_velocity = 0;

    static int8_t use_debug = 0;

    static std::vector<int32_t> detected_sign_array;

    static std::vector<int32_t> debug_dead_end = {
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,
    };

    static std::vector<int32_t> debug_zebracros_tanpa_sign = {
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
    };

    static std::vector<int32_t> debug_zebracros_tanpa_sign_tanpa_putih = {
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
    };

    static std::vector<int> sign_array_debug = {
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
    };

    static std::vector<int> sign_array_debug_default = {

        // ARUCO_TURN_RIGHT,
        // ARUCO_TURN_LEFT,
        // ARUCO_TURN_LEFT,
        // ARUCO_FORWARD,

        // // ARUCO_FORWARD,
        // ARUCO_TURN_LEFT,
        // ARUCO_TURN_RIGHT,
        // ARUCO_TURN_RIGHT,
        // ARUCO_TURN_RIGHT,
        // ARUCO_FORWARD,
        // ARUCO_TURN_RIGHT,
        // ARUCO_TURN_LEFT,
        // ARUCO_STOP,

        ARUCO_TURN_RIGHT,
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,

        // obs

        ARUCO_FORWARD,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_LEFT,
        ARUCO_STOP,

    };

    static std::vector<int> sign_array_debug2_left = {
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
        ARUCO_TURN_LEFT,
    };

    static std::vector<int> sign_array_debug2_right = {
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,
        ARUCO_TURN_RIGHT,
    };

    static std::vector<int> sign_array_debug2_forward = {
        ARUCO_FORWARD,
        ARUCO_FORWARD,
        ARUCO_FORWARD,
        ARUCO_FORWARD,
        ARUCO_FORWARD,
        ARUCO_FORWARD,
        ARUCO_FORWARD,
    };

    if (debug_mode2_ == 0)
    {
        use_debug = 0;
    }
    else if (debug_mode2_ == 1)
    {
        use_debug = 1;
        sign_array_debug = sign_array_debug2_left;
    }
    else if (debug_mode2_ == 2)
    {
        use_debug = 1;
        sign_array_debug = sign_array_debug2_right;
    }
    else if (debug_mode2_ == 3) ///
    {                           ////
        use_debug = 1;
        sign_array_debug = sign_array_debug2_forward;
    }
    else if (debug_mode2_ == 4)
    {
        use_debug = 1;
        sign_array_debug = sign_array_debug_default;
    }

    static int idx_sign_detected = 0;
    static int idx_zebracross_tanpa_sign = 0;
    static int idx_dead_end = 0;

    if (idx_dead_end > debug_dead_end.size() - 1)
        idx_dead_end = 0; // Reset index dead end jika sudah mencapai akhir

    if (idx_sign_detected > sign_array_debug.size() - 1)
        idx_sign_detected = 0; // Reset index sign detected jika sudah mencapai akhir

    if (idx_sign_detected > sign_array_debug.size() - 1)
        use_debug = 0;

    use_debug = 0;
    //! ==========================
    //! ==========================

    //? LOGIC BUTTON
    // if (button_1)
    // {
    //     logger.info("============ button_1 ==============");
    //     urban_fsm.value = 999;
    // }

    // if (button_2)
    // {
    //     logger.info("============ button_2 ==============");
    // }

    float offset_jarak_sign_pole = offset_jarak_sign_pole_; // Offset jarak dari sign pole

    static double start_time_berhenti = 0;
    static float travel_dist_after_stop = 0;
    static int16_t final_sign_detected_status = -1;
    static float pos_enc_stop = 0;
    static float expected_target_steering = 0;
    static float prev_expected_target_steering = 0;
    static float curr_gyro_deg = 0;
    static float filtered_gyro_deg = 0;
    static float last_gyro_stop = 0;
    static float last_dist_zebracross_kiri = 0;
    static float max_encoder_belok = 0;
    static float prev_dist_putih = 0;

    float target_vel_otomatis = velocity_jalan_otomatis;
    float target_putih_x = (urban_data.pos_robot_px_x - urban_data.pos_target_px_x) / urban_data.meter_to_pixel;
    float target_putih_y = (urban_data.pos_robot_px_y - urban_data.pos_target_px_y) / urban_data.meter_to_pixel;
    float dist_target_putih = pythagoras(target_putih_x, target_putih_y, 0, 0);
    float angle_target_putih = urban_data.target_angle_putih DEG2RAD;
    float angle_target_ungu = urban_data.target_angle_ungu DEG2RAD;
    float angle_target_sign = atan2(urban_data.centroid_sign_x + offset_jarak_sign_pole, urban_data.centroid_sign_y);
    float dist_near_zebracross = urban_data.dist_near_zebracross;
    float dist_near_zebracross_vertical = urban_data.dist_near_zebracross_vertical;
    float dist_near_zebracross_horizontal = urban_data.dist_near_zebracross_horizontal;
    float centroid_sign[2] = {urban_data.centroid_sign_x, urban_data.centroid_sign_y};
    float dist_zebra_cross_kiri = urban_data.dist_near_zebracross_vertical_kiri < 1.0 ? urban_data.dist_near_zebracross_vertical_kiri : 99.0;
    float dist_zebra_cross_kanan = urban_data.dist_near_zebracross_vertical_kanan < 1.0 ? urban_data.dist_near_zebracross_vertical_kanan : 99.0;
    float dist_sign_pole = pythagoras(centroid_sign[0], centroid_sign[1], 0, 0);
    float obs_pos_x = urban_data.centroid_obs_x;
    float obs_pos_y = urban_data.centroid_obs_y;
    float jarak_ke_pertigaan = urban_data.jarak_ke_pertigaan;

    uint8_t target_putih_valid = (urban_data.pos_target_px_x != 0 && urban_data.pos_target_px_y != 0) && (dist_target_putih < 0.85 && dist_target_putih > 0.05);
    uint8_t target_edge_valid = (fabs(urban_data.target_angle_ungu) < 999);
    uint8_t target_edge_valid_stop = (fabs(urban_data.target_angle_ungu) < 0.2);
    uint8_t zebracros_valid = (dist_near_zebracross < 0.8 && dist_near_zebracross > 0.1);
    uint8_t zebracros_vertical_valid = (dist_near_zebracross_vertical < 1.0 && dist_near_zebracross_vertical > 0.1);
    uint8_t zebracros_horizontal_valid = (dist_near_zebracross_horizontal < 0.8 && dist_near_zebracross_horizontal > 0.1);
    uint8_t sign_pole_valid = (centroid_sign[0] != 0 && centroid_sign[1] != 0) && (pythagoras(centroid_sign[0], centroid_sign[1], 0, 0) < 0.9);
    uint8_t sign_pole_valid_running = (centroid_sign[0] != 0 && centroid_sign[1] != 0) && (pythagoras(centroid_sign[0], centroid_sign[1], 0, 0) < 1.6);
    uint8_t mask_jalan_bocor = urban_data.mask_jalan_bocor;
    uint8_t ada_obs = (obs_pos_x != 0 && obs_pos_y != 0 && pythagoras(obs_pos_x, obs_pos_y, 0, 0) < 1 && pythagoras(obs_pos_x, obs_pos_y, 0, 0) > 0.3);
    uint8_t ada_pertigaan = urban_data.ada_pertigaan;

    // logger.info("pythagoras(centroid_sign[0], centroid_sign[1], 0, 0: %.2f", pythagoras(centroid_sign[0], centroid_sign[1], 0, 0));

    static uint8_t prev_ada_obs = ada_obs;
    static uint8_t zebracross_tanpa_sign = 0;
    static uint8_t zebracross_tanpa_sign_tanpa_putih = 0;
    static uint8_t jalan_berkelok = 0;
    static uint8_t jalan_bocor = 0;
    static uint8_t prev_mask_jalan_bocor = mask_jalan_bocor;
    static uint8_t ada_zebracross_didepan = 0;
    static int16_t cntr_steering_benar = 0;
    static int16_t cntr_jalan_lurus = 0;
    static uint8_t status_benar_case_0 = 0;
    static uint8_t status_berhenti = 0; // 0 = jalan, 1 = berhenti karena zebracross horizontal, 2 = berhenti karena zebracross vertikal, 3 = berhenti karena putih, 4 = berhenti karena rambu
    static uint8_t status_berbelok = 0; // 0 = tidak berbelok, 1 = belok karena putih, 2 = belok karena encoder
    static uint8_t status_steering = 0; // 0 = tidak steering, 1 = steering karena putih, 2 = steering karena ungu, 3 = steering karena hardcode, 4 = steering karena rambu
    static uint8_t first_time = 0;
    // -- Temporary variables
    float max_encoder_maju = 0.53;
    float min_jarak_ke_putih_ = 0.3;
    float max_delta_derajat_gyro = 80;

    if (prev_mask_jalan_bocor == 0 && mask_jalan_bocor != 0)
        jalan_bocor = !jalan_bocor; // Set jalan bocor jika mask berubah dari 0 ke 1

    if (urban_fsm.value == 0)
    {
        if (urban_data.jalan_berkelok)
        {
            jalan_berkelok = 1;   // Set jalan berkelok jika data jalan berkelok
            cntr_jalan_lurus = 0; // Reset counter jalan lurus
        }
        else
        {
            cntr_jalan_lurus++;
            if (cntr_jalan_lurus > 300)
            {
                jalan_berkelok = 0;   // Reset jalan berkelok jika sudah cukup
                cntr_jalan_lurus = 0; // Reset counter jalan lurus
            }
        }
    }
    else
    {
        cntr_jalan_lurus = 0; // Reset counter jalan lurus
        jalan_berkelok = 0;   // Reset jalan berkelok jika tidak ada data
    }

    if (oto)
        target_velocity = target_vel_otomatis;
    else
        target_velocity = vx;

    curr_gyro_deg += fb_final_vel_dxdydo[2] * 0.02 RAD2DEG; // Update gyro based on the angular velocity

    // if (debug_mode_ == 0)
    // {
    //     logger.info("Urban2: %d -> %d (%d) | st: %.2f, v: %.2f || status: %d % d % d - %d %d ||\n|| valid: %d %d - %d %d %d - %d || zebra: %d || enc: %.2f -> %.2f || gyro: %.2f %.2f %.2f ",
    //                 urban_fsm.value, sign_detected_status, final_sign_detected_status,
    //                 target_steering, target_velocity,
    //                 status_steering, status_berhenti, status_berbelok, jalan_berkelok, jalan_bocor,
    //                 target_putih_valid, target_edge_valid, zebracros_valid, zebracros_vertical_valid, zebracros_horizontal_valid, sign_pole_valid,
    //                 ada_zebracross_didepan,
    //                 travel_dist_after_stop, max_encoder_belok, curr_gyro_deg, filtered_gyro_deg, last_gyro_stop);
    // }
    // logger.info("dist_zebra_cross_kiri: %.2f, dist_zebra_cross_kanan: %.2f ", dist_zebra_cross_kiri, dist_zebra_cross_kanan);

    static float enc_menghindar_obs = enc_meter;

    if (ada_obs)
    {
        //! ==============
        //! CASE BAHAYA
        //! ==============
        //* INTERUPT OBSTACLE LANGSUNG

        urban_fsm.value = 0;
        sign_detected_status = -1;
        final_sign_detected_status = -1;
        enc_menghindar_obs = enc_meter;

        target_steering = atan2(obs_pos_x + 0.45, obs_pos_y); // Set steering towards the obstacle
        logger.warn("INTERUPT OBSTACLE DETECTED: %.2f, %.2f", obs_pos_x, obs_pos_y);

        manual_motion(target_velocity, 0, target_steering);
        prev_ada_obs = 1;
        detected_sign_array.clear(); // Clear the detected sign array
        use_debug = 0;

        return;
    }

    if ((fabs(enc_meter - enc_menghindar_obs) < 0.05) && prev_ada_obs)
    {
        logger.info("AAAAAAAAAAAAAAAA");
        manual_motion(target_velocity, 0, 0);
        return;
    }
    else
    {
        prev_ada_obs = 0;
    }

    switch (urban_fsm.value)
    {
    case 0:
        curr_gyro_deg = 0; // Reset gyro when entering state 0

        //* BUFFER ADA ZEBRACROSS DI DEPAN
        if (zebracros_horizontal_valid && dist_near_zebracross_horizontal == dist_near_zebracross)
            ada_zebracross_didepan = 1; // Set flag if zebra cross is detected in front

        //* GERAK MENGIKUTI TARGET JIKA VALID (PRIORITAS TARGET PUTIH > TARGET UNGU)
        if (target_putih_valid /*&& !jalan_berkelok*/)
        {
            status_steering = 1; // Steering karena target putih valid
            expected_target_steering = angle_target_putih;
        }
        else if (sign_pole_valid && (pythagoras(centroid_sign[0], centroid_sign[1], 0, 0) > 0.2))
        {
            status_steering = 4; // Steering karena rambu valid
            expected_target_steering = angle_target_sign;
        }
        else if (target_edge_valid)
        {
            status_steering = 2; // Steering karena target ungu valid
            expected_target_steering = angle_target_ungu;
        }
        else
        {
            expected_target_steering = 0; // Reset expected steering if no valid target
        }

        if (sign_detected_status == ARUCO_STOP && sign_pole_valid_running)
        {
            status_steering = 4; // Steering karena rambu valid
            expected_target_steering = angle_target_sign;
        }

        //* PERHALUS PERGERAKAN STEERING
        if (fabs(expected_target_steering) > (12 DEG2RAD))
        {
            if (expected_target_steering < target_steering)
                target_steering -= 0.01; // Adjust steering to the left
            else if (expected_target_steering > target_steering)
                target_steering += 0.01; // Adjust steering to the right
        }
        else //
        {
            target_steering = expected_target_steering; // Use the expected steering angle
        }

        // //! ==============
        // //! CASE BAHAYA
        // //! ==============
        // // //* INTERUPT OBSTACLE
        // if (ada_obs) {
        //     target_steering = atan2(obs_pos_x + 0.3, obs_pos_y); // Set steering towards the obstacle
        //     logger.warn("INTERUPT OBSTACLE DETECTED: %.2f, %.2f", obs_pos_x, obs_pos_y);
        // }

        // //* DETEKSI PEMBENARAN STEERING
        // if (fabs(target_steering) < 0.8 DEG2RAD)
        // {
        //     cntr_steering_benar++;
        //     if (cntr_steering_benar > 80)
        //         status_benar_case_0 = 1; // Set status benar case 0
        // }
        // else
        // {
        //     cntr_steering_benar = 0; // Reset counter if steering is not within the threshold
        // }

        // //* TERJADI ANOMALI, PAKSA STEERING LURUS
        // if (status_benar_case_0 == 1 && fabs(expected_target_steering) > 20 DEG2RAD)
        //     target_steering = 0; // Reset steering if it exceeds the threshold

        //* UPDATE GYRO BERDASARKAN OFFSET ZEBRA CROSS / OFFSET LANE
        if (urban_data.offset_angle != 0)
            curr_gyro_deg = (90 - urban_data.offset_angle);
        else if (urban_data.offset_angle_lane != 0)
            curr_gyro_deg = (90 - urban_data.offset_angle_lane);
        else
            logger.warn("Offset angle is zero, using current gyro value: %.2f", curr_gyro_deg);

        // filtered_gyro_deg = 0.9 * filtered_gyro_deg + 0.1 * curr_gyro_deg; // Apply a simple low-pass filter to gyro data
        filtered_gyro_deg = 0.5 * filtered_gyro_deg + 0.5 * curr_gyro_deg; // Apply a simple low-pass filter to gyro data

        //* UPDATE ENCODER
        pos_enc_stop = enc_meter; // Reset travel distance after stop

        // //* DIPELANKAN SEDIKIT
        // if (dist_near_zebracross_horizontal < (jarak_ke_zebracros_ * 1.4) || fabs(centroid_sign[1]) < 0.3) {
        //     if (oto) {
        //         target_velocity = target_vel_otomatis * 0.85;
        //     }
        // }

        if (jalan_berkelok)
            target_velocity = fminf(target_velocity, min_vel_belokan_); // Set minimum velocity for curved roads

        //* DETEKSI ZEBRA CROSS, JIKA VALID, MAKA BERHENTI
        if (zebracros_horizontal_valid)
        {
            logger.warn("zebracros_horizontal_valid");
            if (dist_near_zebracross_horizontal < jarak_ke_zebracros_)
            {
                urban_fsm.value = 1; // Transition to the next state
                status_berhenti = 1; // Berhenti karena zebra cross horizontal
                start_time_berhenti = time_now;
                break;
            }
        }
        else if (target_putih_valid && !zebracros_horizontal_valid && !sign_pole_valid) //* ZEBRACROSS TIDAK KELIHATAN (BELOK KANAN KHUSUS)
        {
            // logger.warn("target_putih_valid && !zebracros_horizontal_valid && !sign_pole_valid");

            // if (dist_target_putih < min_jarak_ke_putih_) {
            //     last_gyro_stop = filtered_gyro_deg;
            //     status_berhenti = 3; // Berhenti karena target putih
            //     start_time_berhenti = time_now;

            //     // if (dist_zebra_cross_kanan < dist_zebra_cross_kiri)
            //     //     urban_fsm.value = 32; // Transition to the next state
            //     // else
            //     //     urban_fsm.value = 320; // Transition to the next state

            //     {
            //         if (dist_zebra_cross_kanan < dist_zebra_cross_kiri)
            //             final_sign_detected_status = ARUCO_TURN_RIGHT; // Transition to the next state
            //         else
            //             final_sign_detected_status = ARUCO_TURN_LEFT; // Transition to the next state

            //         urban_fsm.value = 2; // Transition to the next state
            //         break; // Exit the switch case
            //     }
            // }
        }
        else if (!target_putih_valid && !zebracros_horizontal_valid && sign_pole_valid)
        {
            logger.warn("Tidak ada putih/zebra: %.2f || sign %d", fabs(centroid_sign[1]), final_sign_detected_status);

            //* KASUS KHUSUS
            if (final_sign_detected_status == -1)
            {
                //* ANGGAPANNYA ADA OBSTACLE
                static float target_steering_prev = 0;
                target_steering = atan2(urban_data.centroid_sign_x + 0.2, urban_data.centroid_sign_y);

                if (fabs(centroid_sign[1]) < 0.3)
                    target_steering = target_steering_prev;
                target_steering_prev = target_steering; // Update previous steering angle
            }
            else
            {
                //! ===================
                //! CASE BERBAHAYA
                //! ===================
                if (final_sign_detected_status == ARUCO_STOP || final_sign_detected_status == ARUCO_NO_ENTRY || final_sign_detected_status == ARUCO_DEAD_END)
                {
                    //* SIGN BERSTATUS NAMUN TIDAK ADA ZEBRCROSS
                    target_steering = angle_target_sign; // Use the expected steering angle

                    if (fabs(centroid_sign[1]) < jarak_ke_sign_pole_)
                    {
                        urban_fsm.value = 13; // Transition to the next state
                        break;
                    }
                }
            }
        }
        else if (zebracros_horizontal_valid && !sign_pole_valid_running && target_putih_valid)
        {
            logger.warn("Zebra cross horizontal valid, but no sign pole detected and target_putih_valid");
            if (dist_near_zebracross_horizontal < jarak_ke_zebracros_)
            {
                // zebracross_tanpa_sign = 1;
                urban_fsm.value = 1; // Transition to the next state
                status_berhenti = 1; // Berhenti karena zebra cross horizontal
                start_time_berhenti = time_now;
                break;
            }
        }
        else if (zebracros_horizontal_valid && !sign_pole_valid_running && !target_putih_valid)
        {
            logger.warn("Zebra cross horizontal valid, but no sign or target putih detected");
            if (dist_near_zebracross_horizontal < jarak_ke_zebracros_)
            {
                urban_fsm.value = 1; // Transition to the next state
                status_berhenti = 1; // Berhenti karena zebra cross horizontal
                start_time_berhenti = time_now;
                break;
            }
        }
        // else if (ada_pertigaan && jarak_ke_pertigaan < 0.4) {
        // // ! ==================================
        // // ! CASE BAHAYA
        // // ! ==================================

        //     final_sign_detected_status == ARUCO_TURN_LEFT;
        //     urban_fsm.value = 2; // Transition to the next state
        //     status_berhenti = 1; // Berhenti karena zebra cross horizontal
        //     start_time_berhenti = time_now;
        //     break;
        // }
        logger.info("ada pertigaan kosong %.2f | %d", jarak_ke_pertigaan, ada_pertigaan);

        //! ==================================
        //! CASE BAHAYA (GUNAKAN JIKA DI AWAL START BERADA DI TEMPAT TIDAK ADA ZEBRACROSS)
        //! ==================================
        if (sign_pole_valid_running && dist_near_zebracross_horizontal > 0.5)
        {
            if (first_time)
            {
                logger.warn("sign_pole_valid_running %.2f", dist_sign_pole);
                if (sign_pole_valid && dist_sign_pole < 0.3 && final_sign_detected_status != -1)
                {
                    urban_fsm.value = 2;      // Transition to the next state
                    pos_enc_stop = enc_meter; // Reset travel distance after stop
                    start_time_berhenti = time_now;
                    first_time = 0;
                    break;
                }
            }
        }
        //! ==================================

        //! ==================================
        //! CASE BAHAYA
        //! ==================================
        if (!ada_obs)
        {

            if (final_sign_detected_status == ARUCO_STOP || final_sign_detected_status == ARUCO_NO_ENTRY || final_sign_detected_status == ARUCO_DEAD_END)
            {
                if (sign_pole_valid && dist_sign_pole < 0.3)
                {
                    urban_fsm.value = 99; // Transition to the next state
                    break;
                }
            }
        }
        //! ==================================

        //* DETEKSI SIGN
        if (sign_detected_status != -1 /*  && sign_pole_valid */)
        {
            detected_sign_array.push_back(sign_detected_status);
            if (detected_sign_array.size() > 40)
                detected_sign_array.erase(detected_sign_array.begin());
        }

        //* POLLING SIGN STATUS
        if (detected_sign_array.size() >= 20)
        {
            int count = 0;
            for (int i = 0; i < detected_sign_array.size(); i++)
                if (detected_sign_array[i] == detected_sign_array[0])
                    count++;
            if (count >= 19)
                final_sign_detected_status = detected_sign_array[10]; // Set final sign detected status
        }

        if (use_debug)
        {
            logger.error("========= use_debug =============: %d", idx_sign_detected);
            final_sign_detected_status = sign_array_debug[idx_sign_detected]; // Set final sign detected status from hardcode array
        }

        logger.warn("final_sign_detected_status: %d", final_sign_detected_status);

        // //* UPDATE PREVIOUS EXPECTED STEERING ANGLE
        // prev_expected_target_steering = expected_target_steering; // Update previous expected steering angle

        break;
    case 1:
        //* BERHENTI KARENA ADA ZEBRA CROSS
        target_velocity = 0;
        // final_sign_detected_status = ARUCO_TURN_LEFT;

        //* UPDATE GYRO BERDASARKAN OFFSET ZEBRA CROSS / OFFSET LANE
        if (urban_data.offset_angle != 0)
            curr_gyro_deg = (90 - urban_data.offset_angle);
        else if (urban_data.offset_angle_lane != 0)
            curr_gyro_deg = (90 - urban_data.offset_angle_lane);

        // filtered_gyro_deg = 0.9 * filtered_gyro_deg + 0.1 * curr_gyro_deg; // Apply a simple low-pass filter to gyro data
        filtered_gyro_deg = 0.5 * filtered_gyro_deg + 0.5 * curr_gyro_deg; // Apply a simple low-pass filter to gyro data

        last_gyro_stop = filtered_gyro_deg;
        //
        logger.info("fsm: %d || %d %d %d", urban_fsm.value, zebracross_tanpa_sign, zebracross_tanpa_sign_tanpa_putih, use_debug);

        //* DETEKSI SIGN
        if (sign_detected_status != -1 && sign_pole_valid)
        {
            detected_sign_array.push_back(sign_detected_status);
            if (detected_sign_array.size() > 40)
                detected_sign_array.erase(detected_sign_array.begin());
        }

        if (!sign_pole_valid && target_putih_valid)
            zebracross_tanpa_sign = 1;
        else
            zebracross_tanpa_sign = 0;

        logger.warn("final_sign_detected_status: %d", final_sign_detected_status);

        // if (!sign_pole_valid_running && !target_putih_valid) {
        //     zebracross_tanpa_sign_tanpa_putih = 1;
        // } else {
        //     zebracross_tanpa_sign_tanpa_putih = 0;
        // }

        //* TUNGGU SELAMA 3 DETIK
        if (time_now - start_time_berhenti > 3100)
        {
            // check if detected_sign_array size is 5 and contains the same sign

            //* POLLING SIGN STATUS AUTO
            if (final_sign_detected_status == -1)
            {
                int count = 0;
                for (int i = 0; i < detected_sign_array.size(); i++)
                    if (detected_sign_array[i] == detected_sign_array[0])
                        count++;
                if (count == detected_sign_array.size() && count >= 10)
                    final_sign_detected_status = detected_sign_array[0]; // Set final sign detected status
            }

            //! ==============
            //! CASE BAHAYA
            //! ==============
            // if (zebracross_tanpa_sign == 1) {
            //     logger.error("========= zebracross_tanpa_sign =============");
            //     final_sign_detected_status = debug_zebracros_tanpa_sign[0];
            // }

            //! ==============
            //! CASE BAHAYA
            //! ==============
            // if (zebracross_tanpa_sign_tanpa_putih == 1) {
            //     logger.error("========= zebracross_tanpa_sign_tanpa_putih =============");
            //     final_sign_detected_status = debug_zebracros_tanpa_sign_tanpa_putih[0]; // Set final sign detected status from hardcode array
            // }

            //! ==============
            //! CASE BAHAYA
            //! ==============
            if (dist_sign_pole > 0.5)
            {
                logger.warn("=== ada zebra tapi sign jauh ==");

                if (final_sign_detected_status == -1)
                    final_sign_detected_status = debug_zebracros_tanpa_sign[0];
            }

            if (use_debug)
            {
                logger.error("========= use_debug =============: %d", idx_sign_detected);
                final_sign_detected_status = sign_array_debug[idx_sign_detected]; // Set final sign detected status from hardcode array
            }
            //! ===============
            //! ==============

            //* SIGN TERDETEKSI
            if (final_sign_detected_status != -1)
            {
                urban_fsm.value = 2;      // Transition to the next state after 2 seconds
                pos_enc_stop = enc_meter; // Reset travel distance after stop
            }
            else if (sign_pole_valid && (time_now - start_time_berhenti > 5100))
            {
                logger.warn("Tidak ada sign terdeteksi, mundur sedikit");
                //* mundur sedikit
                urban_fsm.value = 12;     // Transition to the next state after 2 seconds
                pos_enc_stop = enc_meter; // Reset travel distance after stop
            }
        }

        if (debug_mode_ == 2)
            logger.info("last %.2f > curr %.2f > filter %.2f || target: %.2f - %.2f",
                        fabs(last_gyro_stop), curr_gyro_deg, filtered_gyro_deg,
                        target_steering, target_velocity);

        break;
    case 11: //* CASE ANOMALY ZEBRACROSS TANPA SIGN

        if (zebracros_vertical_valid && dist_zebra_cross_kanan < 1.0)
        {
            //* ADA BELOKAN KE KIRI
            final_sign_detected_status = ARUCO_TURN_LEFT; // Set final sign detected status to turn left
        }
        else if (zebracros_vertical_valid && dist_zebra_cross_kiri < 1.0)
        {
            //* ADA BELOKAN KE KANAN
            final_sign_detected_status = ARUCO_TURN_RIGHT; // Set final sign detected status to turn right
        }
        else
        {
            final_sign_detected_status = debug_zebracros_tanpa_sign[0]; // Set final sign detected status from hardcode array
        }

        pos_enc_stop = enc_meter; // Reset travel distance after stop
        urban_fsm.value = 2;      // Transition to the next state

        break;
    case 12:
        //* MUNDUR SEDIKIT
        target_velocity = -0.1; // Set target velocity to reverse
        target_steering = 0;    // Set target steering to straight

        travel_dist_after_stop = fabs(enc_meter - pos_enc_stop); // Calculate travel distance after stop

        if (travel_dist_after_stop > 0.2)
        {
            target_velocity = 0; // Stop if travel distance is greater than 0.1
            urban_fsm.value = 0; // Transition to the next state
            idx_sign_detected++;
            idx_zebracross_tanpa_sign++;
            idx_dead_end++;
            detected_sign_array.clear(); // Clear the detected sign array
            sign_detected_status = -1;
            final_sign_detected_status = -1;
            break;
        }

        break;
    case 13: //* CASE SIGN TANPA ZEBRACROSS

        if (use_debug)
            final_sign_detected_status = sign_array_debug[idx_sign_detected]; // Set final sign detected status from hardcode array

        if (final_sign_detected_status != -1)
        {
            pos_enc_stop = enc_meter; // Reset travel distance after stop
            last_gyro_stop = 0;
            urban_fsm.value = 2; // Transition to the next state
            status_berhenti = 4; // Berhenti karena rambu
            start_time_berhenti = time_now;
        }

        //! JIKA DEAD END KHUSUS
        // if (final_sign_detected_status == ARUCO_DEAD_END)
        // {
        //     urban_fsm.value = 33; // Transition to the next state
        //     break;
        // }

        if (final_sign_detected_status == ARUCO_STOP || final_sign_detected_status == ARUCO_NO_ENTRY || final_sign_detected_status == ARUCO_DEAD_END)
        {
            urban_fsm.value = 99; // Transition to the next state
            break;
        }

        break;
    case 2: //* CASE BELOK MENGIKUTI RAMBU
        if (use_debug)
            final_sign_detected_status = sign_array_debug[idx_sign_detected]; // Set final sign detected status from hardcode array

        logger.warn("final_sign_detected_status: %d", final_sign_detected_status);

        //* RESET ADA ZEBRACROSS DI DEPAN
        ada_zebracross_didepan = 0;

        if (final_sign_detected_status == ARUCO_TURN_LEFT)
        {
            max_encoder_maju = encoder_maju_kiri_;
            min_jarak_ke_putih_ = min_jarak_putih_kiri_;
        }
        else if (final_sign_detected_status == ARUCO_TURN_RIGHT)
        {
            max_encoder_maju = encoder_maju_kanan_;
            min_jarak_ke_putih_ = min_jarak_putih_kanan_;
        }
        else if (final_sign_detected_status == ARUCO_FORWARD)
        {
            target_velocity = 0;
            max_encoder_maju = 0.0;
            min_jarak_ke_putih_ = 9999.0;
            urban_fsm.value = 31; // Transition to the next state
            break;                // Stop processing further in this case
        }
        else if (final_sign_detected_status == ARUCO_STOP)
        {
            target_velocity = 0;
            max_encoder_maju = 0.0;
            min_jarak_ke_putih_ = 9999.0;
            urban_fsm.value = 100; // Transition to the next state
            break;                 // Stop processing further in this case
        }
        else if (final_sign_detected_status == ARUCO_DEAD_END)
        {
            target_velocity = 0;
            max_encoder_maju = 0.0;
            min_jarak_ke_putih_ = 9999.0;
            urban_fsm.value = 100; // Transition to the next state
            break;                 // Stop processing further in this case
        }
        else if (final_sign_detected_status == ARUCO_NO_ENTRY)
        {
            target_velocity = 0;
            max_encoder_maju = 0.0;
            min_jarak_ke_putih_ = 9999.0;
            urban_fsm.value = 100; // Transition to the next state
            break;                 // Stop processing further in this case
        }

        //* GERAK MENGIKUTI TARGET JIKA VALID (PRIORITAS TARGET UNGU > TARGET PUTIH)
        if (target_putih_valid)
        {
            status_steering = 1; // Steering karena target putih valid
            expected_target_steering = angle_target_putih;
        }
        else
        {
            status_steering = 2; // Steering karena target ungu valid
            expected_target_steering = 0.5 DEG2RAD;
        }

        //* PERHALUS PERGERAKAN STEERING
        if (fabs(expected_target_steering) > 15 DEG2RAD)
        {
            if (expected_target_steering < target_steering)
                target_steering -= 0.009; // Adjust steering to the left
            else if (expected_target_steering > target_steering)
                target_steering += 0.009; // Adjust steering to the right
        }
        else
        {
            target_steering = expected_target_steering; // Use the expected steering angle
        }
        last_dist_zebracross_kiri = dist_zebra_cross_kiri; // Update last seen zebra cross distance

        if (fabs(prev_dist_putih - urban_data.dist_putih_meter) > 0.1)
            logger.error("========= ANOMALI ==========");
        logger.info("%.2f", fabs(prev_dist_putih - urban_data.dist_putih_meter));

        //* PILIH TARGET UNTUK BELOK
        if (target_putih_valid) //* DENGAN TARGET PUTIH
        {
            if (urban_data.dist_putih_meter < min_jarak_ke_putih_)
            {
                status_berbelok = 1;
                urban_fsm.value = 3;      // Transition to the next state
                pos_enc_stop = enc_meter; // Reset travel distance after stop
                break;
            }
        }
        else //* DENGAN TARGET ENCODER
        {
            travel_dist_after_stop = enc_meter - pos_enc_stop; // Calculate travel distance after stop
            if (travel_dist_after_stop > max_encoder_maju)
            {
                status_berbelok = 2;
                urban_fsm.value = 3;      // Transition to the next state
                pos_enc_stop = enc_meter; // Reset travel distance after stop
                break;
            }
        }

        // if (debug_mode_)
        // {
        //     logger.info("%d || enc: %.2f > %.2f || gyro: %.2f > %.2f || target: %.2f - %.2f ",
        //                 urban_fsm.value,
        //                 travel_dist_after_stop, max_encoder_belok,
        //                 fabs(last_gyro_stop), filtered_gyro_deg,
        //                 target_steering, target_velocity);
        // }

        break;
    case 3: //* CASE BELOK
    {
        first_time = 0;

        int8_t ungu_belok_valid = 0;
        int8_t ungu_belok_valid_2 = 0;
        float normal_steering = 0;

        // logger.info("======== %d || %.2f [%d]", sign_pole_valid_running, angle_target_sign, sign_detected_status);

        travel_dist_after_stop = enc_meter - pos_enc_stop;
        last_gyro_stop += fb_final_vel_dxdydo[2] * 0.02 RAD2DEG; // Update gyro based on the angular velocity
        status_steering = 3;
        // Steering karena hardcode

        if (final_sign_detected_status == ARUCO_TURN_LEFT)
        {
            max_encoder_belok = encoder_belok_kiri_;
            target_steering = derajat_steering_kiri_ DEG2RAD;
            max_delta_derajat_gyro = derajat_gyro_kiri_;
            normal_steering = derajat_steering_kiri_ DEG2RAD;

            ungu_belok_valid = (urban_data.target_angle_ungu > 0);
        }
        else if (final_sign_detected_status == ARUCO_TURN_RIGHT)
        {
            max_encoder_belok = encoder_belok_kanan_;
            target_steering = derajat_steering_kanan_ DEG2RAD;
            max_delta_derajat_gyro = derajat_gyro_kanan_;
            ungu_belok_valid = (urban_data.target_angle_ungu < 0);
            ungu_belok_valid_2 = (urban_data.target_angle_ungu > 0);

            normal_steering = derajat_steering_kanan_ DEG2RAD;
        }
        else if (final_sign_detected_status == ARUCO_FORWARD)
        {
            urban_fsm.value = 31; // Transition to the next state
            break;
        }

        // logger.info("last gyro: %.2f - %.2f - %.2f", fabs(last_gyro_stop), max_delta_derajat_gyro, fb_final_vel_dxdydo[2] * 0.02 RAD2DEG);
        // logger.info("last seen zebra kiri: %.2f - %.2f | status: %d", last_dist_zebracross_kiri, dist_zebra_cross_kiri, status_berbelok);

        //* PAKAI GYRO
        if (debug_mode_ == 2 || debug_mode_ == 0)
        {
            if (fabs(last_gyro_stop) > max_delta_derajat_gyro)
            {
                if (ungu_belok_valid)
                {
                    status_steering = 2; // Steering karena target ungu valid
                    target_steering = angle_target_ungu;
                }
                else
                {
                    target_steering = normal_steering;
                }

                if (sign_pole_valid_running && (sign_detected_status == ARUCO_STOP))
                {
                    logger.info("========= MASUK SINI ==========");
                    target_steering = angle_target_sign;
                    urban_fsm.value = 99;
                    break;
                }
            }

            if ((fabs(last_gyro_stop) > last_gyro_angle_))
            {
                // reset sign status

                sign_detected_status = -1;
                final_sign_detected_status = -1;
                detected_sign_array.clear(); // Clear the detected sign array
                curr_gyro_deg = 0;           // Reset current gyro degree
                urban_fsm.value = 0;         // Transition to the next state
                idx_sign_detected++;
                idx_zebracross_tanpa_sign++;
                idx_dead_end++;
                detected_sign_array.clear(); // Clear the detected sign array
                sign_detected_status = -1;
                final_sign_detected_status = -1;
                break;
            }
        }

        //* PAKAI ENCODER
        if (debug_mode_ == 1 || debug_mode_ == 0)
        {
            //! SAFETY ENCODER
            // logger.info("travel_dist_after_stop: %.2f - %.2f", travel_dist_after_stop, max_encoder_belok);
            if (travel_dist_after_stop > max_encoder_belok)
            {
                if (ungu_belok_valid)
                {
                    status_steering = 2; // Steering karena target ungu valid
                    target_steering = angle_target_ungu;
                }
                else
                {
                    target_steering = normal_steering;
                }

                if (sign_pole_valid_running && (sign_detected_status == ARUCO_STOP))
                {
                    logger.info("========= MASUK SINI ==========");
                    target_steering = angle_target_sign;
                    urban_fsm.value = 99;
                }

                sign_detected_status = -1;
                final_sign_detected_status = -1;
                detected_sign_array.clear(); // Clear the detected sign array
                curr_gyro_deg = 0;           // Reset current gyro degree
                urban_fsm.value = 0;         // Transition to the next state
                idx_sign_detected++;
                idx_zebracross_tanpa_sign++;
                idx_dead_end++;
                detected_sign_array.clear(); // Clear the detected sign array
                sign_detected_status = -1;
                final_sign_detected_status = -1;
                break;
            }

            if (travel_dist_after_stop > max_encoder_belok * 0.85)
            {
                if (ungu_belok_valid_2)
                {
                    status_steering = 2; // Steering karena target ungu valid
                    target_steering = angle_target_ungu;
                }
                else
                {
                    target_steering = normal_steering;
                }

                if (sign_pole_valid_running && (sign_detected_status == ARUCO_STOP))
                {
                    logger.info("========= MASUK SINI ==========");
                    target_steering = angle_target_sign;
                    urban_fsm.value = 99;
                }
            }
        }

        logger.info("target steering %.2f || %.2f || %d", target_steering, urban_data.target_angle_ungu, ungu_belok_valid);

        std_msgs::msg::Float32 msg_curr_gyro_deg;
        msg_curr_gyro_deg.data = fabs(last_gyro_stop);
        pub_curr_gyro_deg->publish(msg_curr_gyro_deg);

        if (debug_mode_ != 0)
            logger.info("fsm: %d [%d] [%d] || enc: %.2f > %.2f || gyro: %.2f > %.2f > %.2f || target: %.2f - %.2f",
                        urban_fsm.value, debug_mode_, final_sign_detected_status,
                        travel_dist_after_stop, max_encoder_belok,
                        fabs(last_gyro_stop), max_delta_derajat_gyro, last_gyro_angle_,
                        target_steering, target_velocity);

        break;
    }
    case 31:                 //* CASE LURUS
        target_steering = 0; // Set steering to zero for going straight
        status_steering = 3; // Steering karena hardcode
        travel_dist_after_stop = enc_meter - pos_enc_stop;

        if (target_putih_valid && dist_target_putih > 0.1)
        {
            target_steering = (angle_target_putih); // Use the target putih angle if valid
            status_steering = 1;                    // Steering karena target putih valid
        }

        if (target_putih_valid && dist_target_putih > 0.4)
            pos_enc_stop = enc_meter; // Reset travel distance after stop

        logger.info("travel_dist_after_stop: %.2f", travel_dist_after_stop);

        if (!zebracros_valid && (travel_dist_after_stop > encoder_maju_lurus_))
        {
            // reset sign status
            sign_detected_status = -1;
            final_sign_detected_status = -1;
            detected_sign_array.clear(); // Clear the detected sign array
            curr_gyro_deg = 0;           // Reset current gyro degree
            urban_fsm.value = 0;         // Transition to the next state
            idx_sign_detected++;
            idx_zebracross_tanpa_sign++;
            idx_dead_end++;
            detected_sign_array.clear(); // Clear the detected sign array
            sign_detected_status = -1;
            final_sign_detected_status = -1;
            break;
        }

        break;
    case 32: //* CASE BELOK KANAN KHUSUS

        target_steering = derajat_steering_kanan_ DEG2RAD;
        last_gyro_stop += fb_final_vel_dxdydo[2] * 0.02 RAD2DEG; // Update gyro based on the angular velocity

        if (fabs(last_gyro_stop) > derajat_gyro_kanan_)
        {
            if ((urban_data.target_angle_ungu < 0))
            {
                status_steering = 2; // Steering karena target ungu valid
                target_steering = angle_target_ungu;
            }
        }

        if ((fabs(last_gyro_stop) > 80))
        {
            curr_gyro_deg = 0;   // Reset current gyro degree
            urban_fsm.value = 0; // Transition to the next state
            idx_sign_detected++;
            idx_zebracross_tanpa_sign++;
            idx_dead_end++;
            detected_sign_array.clear(); // Clear the detected sign array
            sign_detected_status = -1;
            final_sign_detected_status = -1;
        }
        break;
    case 320:
        target_steering = 32 DEG2RAD;
        last_gyro_stop += fb_final_vel_dxdydo[2] * 0.02 RAD2DEG; // Update gyro based on the angular velocity

        if (fabs(last_gyro_stop) > derajat_gyro_kanan_)
        {
            if ((urban_data.target_angle_ungu > 0))
            {
                status_steering = 2; // Steering karena target ungu valid
                target_steering = angle_target_ungu;
            }
        }

        if ((fabs(last_gyro_stop) > 78))
        {
            curr_gyro_deg = 0;   // Reset current gyro degree
            urban_fsm.value = 0; // Transition to the next state
            idx_sign_detected++;
            idx_zebracross_tanpa_sign++;
            idx_dead_end++;
            detected_sign_array.clear(); // Clear the detected sign array
            sign_detected_status = -1;
            final_sign_detected_status = -1;
        }
        break;

    case 33: //* CASE DEAD END dan NO ENTRY
    {
        //* GERAK MENGIKUTI TARGET JIKA VALID (PRIORITAS TARGET UNGU > TARGET PUTIH)
        if (target_putih_valid)
        {
            status_steering = 1; // Steering karena target putih valid
            expected_target_steering = angle_target_putih;
        }
        else if (target_edge_valid)
        {
            status_steering = 2; // Steering karena target ungu valid
            expected_target_steering = angle_target_ungu;
        }

        //* PERHALUS PERGERAKAN STEERING
        if (fabs(expected_target_steering) > 15 DEG2RAD)
        {
            if (expected_target_steering < target_steering)
                target_steering -= 0.009; // Adjust steering to the left
            else if (expected_target_steering > target_steering)
                target_steering += 0.009; // Adjust steering to the right
        }
        else
        {
            target_steering = expected_target_steering; // Use the expected steering angle
        }

        travel_dist_after_stop = enc_meter - pos_enc_stop;
        final_sign_detected_status = debug_dead_end[0]; // Set final sign detected status

        if (travel_dist_after_stop > encoder_maju_dead_end_)
        {
            last_gyro_stop = 0;
            urban_fsm.value = 2; // Transition to the next state
        }

        break;
    }
    case 99: //* CASE STOP
        target_steering = 0;

        if (sign_pole_valid_running)
            target_steering = angle_target_sign; // Use the target sign angle if valid

        if (dist_sign_pole < 0.3 && sign_pole_valid)
            urban_fsm.value = 100; // Transition to the next state

        break;
    case 100:
        target_velocity = 0;
        target_steering = 0;

        break;
    case 999:
        ada_zebracross_didepan = 0;
        zebracross_tanpa_sign = 0;
        cntr_steering_benar = 0;
        status_benar_case_0 = 0;
        status_berhenti = 0; // 0 = jalan, 1 = berhenti karena zebracross horizontal, 2 = berhenti karena zebracross vertikal, 3 = berhenti karena putih, 4 = berhenti karena rambu
        status_berbelok = 0; // 0 = tidak berbelok, 1 = belok karena putih, 2 = belok karena encoder
        status_steering = 0; // 0 = tidak steering, 1 = steering karena putih, 2 = steering karena ungu, 3 = steering karena hardcode, 4 = steering karena rambu
        last_gyro_stop = 0;
        sign_detected_status = -1;
        final_sign_detected_status = -1;
        idx_dead_end = 0;
        idx_sign_detected = 0;
        idx_zebracross_tanpa_sign = 0;
        detected_sign_array.clear(); // Clear the detected sign array
        start_time_berhenti = 0;
        travel_dist_after_stop = 0;
        pos_enc_stop = 0;
        expected_target_steering = 0;
        prev_expected_target_steering = 0;
        curr_gyro_deg = 0;
        filtered_gyro_deg = 0;
        last_dist_zebracross_kiri = 0;
        max_encoder_belok = 0;
        jalan_bocor = 0;
        zebracross_tanpa_sign = 0;
        zebracross_tanpa_sign_tanpa_putih = 0;

        prev_mask_jalan_bocor = mask_jalan_bocor;

        urban_fsm.value = 0; // Transition to the next state

        break; //* CASE REENTRY
    }

    //* UPDATE INDEX
    // if ((urban_fsm.prev_value == 3 || urban_fsm.prev_value == 32 || urban_fsm.prev_value == 320 || urban_fsm.prev_value == 32) && urban_fsm.value == 0) {
    //     // Reset variables when entering state 0
    //     idx_sign_detected++;
    //     idx_zebracross_tanpa_sign++;
    //     idx_dead_end++;
    //     detected_sign_array.clear(); // Clear the detected sign array
    //     sign_detected_status = -1;
    //     final_sign_detected_status = -1;
    //     logger.info("Resetting variables for new state 0: idx_sign_detected: %d, idx_zebracross_tanpa_sign: %d, idx_dead_end: %d", idx_sign_detected, idx_zebracross_tanpa_sign, idx_dead_end);
    // }

    urban_fsm.prev_value = urban_fsm.value;
    prev_mask_jalan_bocor = mask_jalan_bocor; // Update previous mask jalan bocor
    prev_dist_putih = urban_data.dist_putih_meter;

    manual_motion(target_velocity, 0, target_steering);

    std_msgs::msg::Int8 msg_buzzer_sign;
    msg_buzzer_sign.data = final_sign_detected_status;
    pub_sign_buzzer->publish(msg_buzzer_sign);

    std_msgs::msg::Int16 msg_state_urban;
    msg_state_urban.data = urban_fsm.value;
    pub_state_urban->publish(msg_state_urban);
}

// void Master::urban_move(float vx, float vy, float wz)
// {

//     urban_move2(vx, vy, wz);
//     return;

//     static MachineState urban_fsm;
//     urban_fsm.reentry(0, 0.5);

//     static std::vector<int> sign_array_debug = {
//         ARUCO_TURN_LEFT,
//         ARUCO_TURN_RIGHT,
//         ARUCO_TURN_RIGHT,
//         ARUCO_TURN_LEFT,
//         ARUCO_TURN_LEFT,
//         ARUCO_TURN_LEFT,
//         ARUCO_TURN_LEFT,
//         ARUCO_TURN_RIGHT,
//         ARUCO_TURN_RIGHT,
//         ARUCO_TURN_RIGHT,
//         ARUCO_TURN_RIGHT,
//         ARUCO_TURN_LEFT,
//         ARUCO_TURN_LEFT,
//         ARUCO_TURN_LEFT,
//         ARUCO_TURN_LEFT,
//         ARUCO_TURN_RIGHT,

//     };

//     double time_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//     static int idx_sign_array = 0;

//     static float expected_target_steering = 0;
//     static float target_steering = 0;
//     static float target_velocity = 0;
//     static float target_putih_x;
//     static float target_putih_y;
//     static float dist_target_putih = 0;
//     static float travel_dist_after_stop = 0;
//     static float pos_enc_stop = 0;
//     static float curr_gyro = 0;
//     static float last_gyro_stop = 0;

//     static int16_t final_sign_detected_status = -1;
//     static int8_t tiang_rambu_detected = -1;
//     static double start_time_berhenti = 0;

//     static int cntr_sign_not_detected = 0;

//     static float prev_dist_target_putih = 0;

//     static std::vector<int> sign_detected_status_history;

//     if (urban_data.pos_target_px_x != 0 && urban_data.pos_target_px_y != 0) {
//         target_putih_x = (urban_data.pos_robot_px_x - urban_data.pos_target_px_x) / urban_data.meter_to_pixel;
//         target_putih_y = (urban_data.pos_robot_px_y - urban_data.pos_target_px_y) / urban_data.meter_to_pixel;

//         dist_target_putih = pythagoras(target_putih_x, target_putih_y, 0, 0);
//     }

//     float angle_target_putih = atan2(target_putih_x, target_putih_y);
//     float angle_target_ungu = urban_data.target_angle_ungu * M_PI / 180.0;

//     float dist_to_sign = pythagoras(urban_data.centroid_sign_x, urban_data.centroid_sign_y, 0, 0);

//     if (dist_to_sign < 1.0 && dist_to_sign > 0.05) {
//         // ada sign terdeteksi
//         tiang_rambu_detected = 1;
//         // logger.info("Urban: ada sign terdeteksi di %.2f %.2f", urban_data.centroid_sign_x, urban_data.centroid_sign_y);
//     }

//     // if (urban_fsm.value != 3) {
//     //     if (sign_detected_status == -1) {
//     //         if (cntr_sign_not_detected++ > 300) {
//     //             sign_detected_status_history.clear();
//     //             final_sign_detected_status = -1; // Reset final sign detected status if no sign is detected for a while
//     //         }
//     //     } else {
//     //         cntr_sign_not_detected = 0; // Reset counter if a sign is detected
//     //     }
//     // } else {
//     //     cntr_sign_not_detected = 0; // Reset counter if a sign is detected
//     // }

//     // if (sign_detected_status_history.size() > 30) {
//     //     sign_detected_status_history.erase(sign_detected_status_history.begin());
//     // }

//     // logger.info("curr gyro: %.2f || %.2f", fb_final_pose_xyo[2] * 180.0 / M_PI, fb_final_vel_dxdydo[2] * 180.0 / M_PI);

//     logger.info("Urban: %d -> %d (%d) | putih: %.2f %.2f | target: %.2f %.2f || dist near clstr: %.2f [%d]", urban_fsm.value, sign_detected_status, final_sign_detected_status, target_putih_x, target_putih_y, urban_data.target_angle_ungu, urban_data.target_angle_ungu, urban_data.dist_near_zebracross, urban_data.berhenti);

//     switch (urban_fsm.value) {
//     case 0:
//         target_velocity = vx;
//         expected_target_steering = angle_target_ungu;
//         final_sign_detected_status = sign_array_debug[idx_sign_array];

//         if (urban_data.dist_putih_meter < 0.8 && urban_data.dist_putih_meter > 0.05) {
//             expected_target_steering = angle_target_putih;
//             // logger.info("=========== steer pakai putih =============");
//         }

//         if (fabs(expected_target_steering) > 15 DEG2RAD) {
//             if (expected_target_steering < target_steering) {
//                 target_steering -= 0.009; // Adjust steering to the left
//             } else if (expected_target_steering > target_steering) {
//                 target_steering += 0.009; // Adjust steering to the right
//             }
//         } else {
//             target_steering = expected_target_steering; // Use the expected steering angle
//         }

//         //? jika ada zebracross, berhenti
//         if (urban_data.dist_near_zebracross_horizontal < jarak_ke_zebracros_) {
//             logger.info("Urban: %d | berhenti di zebra cross", urban_fsm.value);
//             urban_fsm.value = 1; // Transition to the next state
//             start_time_berhenti = time_now;
//         }

//         //? CASE ada belokan namun tidak ada zebracross
//         if (urban_data.dist_near_zebracross_horizontal > 99) {
//             if (urban_data.dist_near_zebracross_vertical < 0.23 || dist_target_putih < 0.2) {
//                 urban_fsm.value = 99;
//                 last_gyro_stop = fb_final_pose_xyo[2] RAD2DEG + (90 - urban_data.offset_angle);
//             }
//         }

//         //? jika tidak ada zebracross, tapi ada tiang rambu
//         // if (urban_data.dist_near_zebracross_horizontal > 99 && urban_data.dist_near_zebracross_vertical > 99 && tiang_rambu_detected == 1)
//         // {
//         //     if (dist_to_sign < 0.5 && dist_to_sign > 0.05)
//         //     {
//         //         logger.info("Urban: %d | ada tiang rambu terdeteksi, berhenti", urban_fsm.value);
//         //         urban_fsm.value = 100; // Transition to the next state
//         //         start_time_berhenti = time_now;
//         //     }
//         //     else
//         //     {
//         //         logger.info("Urban: %d | tidak ada tiang rambu terdeteksi, lanjutkan", urban_fsm.value);
//         //     }
//         // }

//         logger.info("dist_near_zebracross_horizontal: %.2f || dist_near_zebracross_vertical: %.2f || dist_target_putih: %.2f", urban_data.dist_near_zebracross_horizontal, urban_data.dist_near_zebracross_vertical, dist_target_putih);

//         // if (sign_detected_status != -1) {
//         //     final_sign_detected_status = sign_detected_status;
//         //     sign_detected_status_history.push_back(sign_detected_status);
//         // }

//         // if (final_sign_detected_status == 5 || final_sign_detected_status == 0) {
//         //  final_sign_detected_status = -1;
//         // }

//         break;
//     case 1:
//         // target_steering = angle_target_putih;
//         target_velocity = 0;

//         last_gyro_stop = fb_final_pose_xyo[2] RAD2DEG + (90 - urban_data.offset_angle);

//         final_sign_detected_status = ARUCO_TURN_LEFT;

//         // if (sign_detected_status != -1) {
//         //     final_sign_detected_status = sign_detected_status;
//         //     sign_detected_status_history.push_back(sign_detected_status);
//         // }

//         // if (final_sign_detected_status == 5 || final_sign_detected_status == 0) {
//         //     final_sign_detected_status = -1;
//         // }

//         logger.info("Urban: %d | %.2f seconds elapsed, moving to next state", urban_fsm.value, (time_now - start_time_berhenti));
//         if (time_now - start_time_berhenti > 3100) {

//             if (final_sign_detected_status != -1) {
//                 idx_sign_array++;
//                 urban_fsm.value = 2; // Transition to the next state after 2 seconds
//                 pos_enc_stop = enc_meter; // Reset travel distance after stop
//             }
//         }

//         break;
//     case 2: {
//         float max_encoder_maju = 0.53;
//         float min_jarak_ke_putih_ = 0.3;
//         target_velocity = vx;

//         //* steering target putih jika kelihatan
//         if (urban_data.dist_putih_meter < 0.8 && urban_data.dist_putih_meter > 0.05) {
//             // logger.info("=========== steer pakai putih =============");
//             target_steering = angle_target_putih;
//         } else {
//             target_steering = angle_target_ungu;
//         }

//         if (final_sign_detected_status == ARUCO_TURN_LEFT) {

//             max_encoder_maju = encoder_maju_kiri_;
//             min_jarak_ke_putih_ = min_jarak_putih_kiri_;
//         } else if (final_sign_detected_status == ARUCO_TURN_RIGHT) {

//             max_encoder_maju = encoder_maju_kanan_;
//             min_jarak_ke_putih_ = min_jarak_putih_kanan_;
//         } else if (final_sign_detected_status == ARUCO_FORWARD) {

//             max_encoder_maju = encoder_maju_lurus_;
//             min_jarak_ke_putih_ = min_jarak_putih_lurus_;
//         } else if (final_sign_detected_status == ARUCO_STOP) {

//             max_encoder_maju = 0.0;
//             min_jarak_ke_putih_ = 9999.0;
//         }

//         travel_dist_after_stop = enc_meter - pos_enc_stop;
//         logger.info("Travel distance after stop: %.2f > %.2f|| dist putih: %.2f > %.2f", travel_dist_after_stop, max_encoder_maju, dist_target_putih, jarak_ke_putih_);

//         //* jika telah mendekati putih
//         // if ((urban_data.dist_putih_meter > min_jarak_ke_putih_ && urban_data.dist_putih_meter < 0.6) || travel_dist_after_stop < max_encoder_maju)
//         // {
//         //     target_velocity = vx;
//         // }
//         // else
//         // {
//         //     urban_fsm.value = 3;
//         //     pos_enc_stop = enc_meter; // Reset travel distance after stop
//         // }

//         //* jika telah mendekati putih
//         if (urban_data.dist_putih_meter < min_jarak_ke_putih_) {
//             urban_fsm.value = 3;
//             pos_enc_stop = enc_meter; // Reset travel distance after stop
//             break;
//         }

//         //* jika tidak ada putih, tapi ada zebracross, maju dengan encoder
//         if (urban_data.dist_putih_meter > 1.5) {
//             if (travel_dist_after_stop > max_encoder_maju) {
//                 urban_fsm.value = 3; // Transition to the next state
//                 pos_enc_stop = enc_meter; // Reset travel distance after stop
//                 break;
//             }
//         }

//         break;
//     }
//     case 3: //? BERGERAK MENGIKUTI ARUCO
//     {
//         target_velocity = vx;
//         travel_dist_after_stop = enc_meter - pos_enc_stop;
//         // logger.info("Travel distance after stop: %.2f || dist putih: %.2f", travel_dist_after_stop, dist_target_putih);

//         float delta = fabs(last_gyro_stop - fb_final_pose_xyo[2] RAD2DEG);
//         // logger.info("delta gyro: %.2f || last gyro stop: %.2f || curr gyro: %.2f", delta, last_gyro_stop, fb_final_pose_xyo[2] * 180.0 / M_PI);

//         //? =================================================================
//         //? CASE ARUCO TURN LEFT
//         //? =================================================================
//         if (final_sign_detected_status == ARUCO_TURN_LEFT) {

//             target_steering = derajat_steering_kiri_ DEG2RAD;

//             if ((delta > derajat_gyro_kiri_)) {
//                 urban_fsm.value = 0;
//                 idx_sign_detected++;
//                 idx_zebracross_tanpa_sign++;
//                 idx_dead_end++;
//                 detected_sign_array.clear(); // Clear the detected sign array
//                 sign_detected_status = -1;
//                 final_sign_detected_status = -1;
//                 final_sign_detected_status = -1; // Reset final sign detected status
//                 sign_detected_status_history.clear();
//                 break;
//             }
//             logger.info("BELOK KIRI | target steering kiri: %.2f || delta: %.2f > %.2f", target_steering, delta, derajat_gyro_kiri_);
//         }
//         //? =================================================================
//         //? CASE ARUCO TURN RIGHT
//         //? =================================================================
//         else if (final_sign_detected_status == ARUCO_TURN_RIGHT) {

//             if ((delta < derajat_gyro_kanan_)) {
//                 target_steering = derajat_steering_kanan_ DEG2RAD;
//                 // urban_fsm.value = 0;
//                 // sign_detected_status_history.clear();
//                 // final_sign_detected_status = -1; // Reset final sign detected status
//             } else {
//                 target_steering = 0;
//             }

//             if (urban_data.dist_near_zebracross > 1.5) {
//                 // urban_fsm.value = 0;
//                 // idx_sign_detected++;
//                 // idx_zebracross_tanpa_sign++;
//                 // idx_dead_end++;
//                 // detected_sign_array.clear(); // Clear the detected sign array
//                 // sign_detected_status = -1;
//                 // final_sign_detected_status = -1;
//                 // sign_detected_status_history.clear();
//                 // final_sign_detected_status = -1; // Reset final sign detected status
//                 // logger.info("fsm: %d, travel_dist_after_stop: %.2f || dist near zebracross: %.2f", urban_fsm.value, travel_dist_after_stop, urban_data.dist_near_zebracross);
//                 break;
//             }

//             logger.info("BELOK KANAN | target steering kanan: %.2f || delta: %.2f > %.2f", target_steering, delta, derajat_gyro_kanan_);
//         }
//         //? =================================================================
//         //? CASE ARUCO FORWARD
//         //? =================================================================
//         else if (final_sign_detected_status == ARUCO_FORWARD) {

//             target_steering = urban_data.target_angle_ungu DEG2RAD;

//             if (dist_target_putih < 0.8 && dist_target_putih > 0.05) {
//                 // logger.info("=========== steer pakai putih =============");
//                 target_steering = angle_target_putih;
//             }

//             if (travel_dist_after_stop > 0.4 && urban_data.dist_near_zebracross > 1.5) {
//                 // urban_fsm.value = 0;
//                 // idx_sign_detected++;
//                 // idx_zebracross_tanpa_sign++;
//                 // idx_dead_end++;
//                 // detected_sign_array.clear(); // Clear the detected sign array
//                 // sign_detected_status = -1;
//                 // final_sign_detected_status = -1;
//                 // sign_detected_status_history.clear();
//                 // final_sign_detected_status = -1; // Reset final sign detected status
//                 logger.info("fsm: %d, travel_dist_after_stop: %.2f || dist near zebracross: %.2f", urban_fsm.value, travel_dist_after_stop, urban_data.dist_near_zebracross);
//                 break;
//             }
//         }
//         //? ========================================
//         //? CASE ARUCO STOP
//         //? ========================================
//         else if (final_sign_detected_status == ARUCO_STOP) {
//             target_steering = 0;
//             target_velocity = 0;
//         } else {
//             target_steering = 0; // Default steering angle if no sign detected
//         }

//         break;
//     }
//     case 99: {

//         target_velocity = vx;
//         target_steering = -35 DEG2RAD; // Default steering angle for emergency stop

//         float delta = fabs(last_gyro_stop - fb_final_pose_xyo[2] RAD2DEG);

//         if ((delta > 80)) {
//             urban_fsm.value = 0;
//             // idx_sign_detected++;
//             // idx_zebracross_tanpa_sign++;
//             // idx_dead_end++;
//             // detected_sign_array.clear(); // Clear the detected sign array
//             // sign_detected_status = -1;
//             // final_sign_detected_status = -1;
//             // final_sign_detected_status = -1; // Reset final sign detected status
//             logger.info("fsm: %d, travel_dist_after_stop: %.2f || dist near zebracross: %.2f", urban_fsm.value, travel_dist_after_stop, urban_data.dist_near_zebracross);
//             break;
//         }

//         break;
//     }
//     case 100: {
//         //? BERHENTI STOP
//         target_velocity = 0;
//         target_steering = 0; // Default steering angle for emergency stop

//         break;
//     }
//     }

//     urban_fsm.prev_value = urban_fsm.value;

//     manual_motion(target_velocity, 0, target_steering);

//     std_msgs::msg::Int16 msg_state_urban;
//     msg_state_urban.data = urban_fsm.value;
//     pub_state_urban->publish(msg_state_urban);
// }

int8_t Master::control_steering(float vx, float target_steering_local)
{
    const float kp = 1.0;
    const float ki = 0.0;
    const float kd = 0.0;

    static float integral = 0;

    static float prev_calc_steering = target_steering_local;
    static float prev_x = fb_final_pose_xyo[0];
    static float prev_y = fb_final_pose_xyo[1];
    static float distance_traveled = 0;
    static float prev_output_steering = 0;

    static bool initial_move = true; // Flag to skip distance check initially

    float dx = fb_final_pose_xyo[0] - prev_x;
    float dy = fb_final_pose_xyo[1] - prev_y;

    float calc_steering = atan2(dy, dx) - fb_final_pose_xyo[2];
    while (calc_steering < -M_PI)
        calc_steering += 2 * M_PI; // Normalize to [-pi, pi]

    while (calc_steering > M_PI)
        calc_steering -= 2 * M_PI; // Normalize to [-pi, pi]

    float error_steering = target_steering_local - calc_steering;
    while (error_steering < -M_PI)
        error_steering += 2 * M_PI; // Normalize to [-pi, pi]

    while (error_steering > M_PI)
        error_steering -= 2 * M_PI; // Normalize to [-pi, pi]

    float proportional = kp * error_steering;
    integral += ki * error_steering;                              // Integral term
    float derivative = kd * (calc_steering - prev_calc_steering); // Derivative term
    float output_steering = proportional + integral + derivative;

    if (output_steering > profile_max_steering_rad)
        output_steering = profile_max_steering_rad;
    else if (output_steering < -profile_max_steering_rad)
        output_steering = -profile_max_steering_rad;

    // Calculate distance traveled
    distance_traveled += sqrtf(dx * dx + dy * dy);

    // Apply minimal correction if linear movement exceeds 20 cm
    if (!initial_move && distance_traveled > 0.2)
    {
        if (fabs(output_steering) < 0.05) // Minimal correction threshold

            output_steering = 0; // Straighten steering for safety

        distance_traveled = 0; // Reset distance tracker
    }
    else
    {
        initial_move = false;                   // Disable initial move flag after first iteration
        output_steering = prev_output_steering; // Use previous steering if movement is less than 20 cm
    }

    logger.info("Control Steering: %.2f | Target Steering: %.2f | Calc Steering: %.2f | Error Steering: %.2f | Output Steering: %.2f | Distance Traveled: %.2f", target_steering_local, calc_steering, error_steering, output_steering, distance_traveled);
    manual_motion(vx, 0, output_steering);

    prev_x = fb_final_pose_xyo[0];
    prev_y = fb_final_pose_xyo[1];
    prev_calc_steering = calc_steering;
    prev_output_steering = output_steering;
}

int8_t Master::move_right(float vx, float max_counter, float target_theta)
{
    static MachineState fsm;

    static float target_steering = 0.46; // Steering angle for moving left
    static float target_velocity = 0.5;  // Target velocity for moving left

    static float enc_awal = 0;
    static float enc_sekarang = enc_meter;

    fsm.reentry(0, 0.5);

    switch (fsm.value)
    {
    case 0:
        enc_awal = enc_meter;
        fsm.value = 1;
        break;
    case 1:
        enc_sekarang = enc_meter;
        target_steering = -0.57; // Steering angle for moving left
        target_velocity = vx;    // Target velocity for moving left

        // if (enc_sekarang - enc_awal > max_counter) {
        //     fsm.value = 2; // Transition to the next state
        // }

        if (abs(target_theta - (fb_final_pose_xyo[2] * 180 / M_PI)) < 5)
            fsm.value = 2;

        break;
    case 2:
        target_steering = 0; // Steering angle for moving left
        target_velocity = 0;

        break;
    }

    logger.info("Moving right: %d || %.2f -> %.2f : %.2f", fsm.value, enc_awal, enc_sekarang, enc_sekarang - enc_awal);
    manual_motion(target_velocity, 0, target_steering); // Move right with a slight steering angle

    if (fsm.value == 2)
        return 1;
    else
        return 0; // Still in the process of moving right
}

int8_t Master::move_left(float vx, float max_counter, float target_theta)
{
    static MachineState fsm;

    static float target_steering = 0.46; // Steering angle for moving left
    static float target_velocity = 0.5;  // Target velocity for moving left

    static float enc_awal = 0;
    static float enc_sekarang = enc_meter;

    fsm.reentry(0, 0.5);

    switch (fsm.value)
    {
    case 0:
        enc_awal = enc_meter;
        fsm.value = 1;
        break;
    case 1:
        enc_sekarang = enc_meter;
        target_steering = 0.57; // Steering angle for moving left
        target_velocity = vx;   // Target velocity for moving left

        // if (enc_sekarang - enc_awal > max_counter) // Check if the robot has moved 20 cm
        // {
        //     fsm.value = 2; // Transition to the next state
        // }

        if (abs(target_theta - (fb_final_pose_xyo[2] * 180 / M_PI)) < 5)
            fsm.value = 2;

        break;
    case 2:
        target_steering = 0; // Steering angle for moving left
        target_velocity = 0;

        break;
    }

    logger.info("Moving left: %d || %.2f -> %.2f : %.2f", fsm.value, enc_awal, enc_sekarang, enc_sekarang - enc_awal);
    manual_motion(target_velocity, 0, target_steering); // Move right with a slight steering angle

    if (fsm.value == 2)
        return 1;
    else
        return 0; // Still in the process of moving right
}

int8_t Master::forward_move(float vx, float max_counter)
{
    static MachineState fsm;

    static float target_steering = 0.46; // Steering angle for moving left
    static float target_velocity = 0.5;  // Target velocity for moving left

    static float enc_awal = 0;
    static float enc_sekarang = enc_meter;

    fsm.reentry(0, 0.5);

    switch (fsm.value)
    {
    case 0:
        enc_awal = enc_meter;
        fsm.value = 1;
        break;
    case 1:
        enc_sekarang = enc_meter;
        target_steering = 0;  // Steering angle for moving left
        target_velocity = vx; // Target velocity for moving left

        if (enc_sekarang - enc_awal > max_counter) // Check if the robot has moved 20 cm

            fsm.value = 2; // Transition to the next state

        break;
    case 2:
        target_steering = 0; // Steering angle for moving left
        target_velocity = 0;

        break;
    }
}

int8_t Master::obstacle_avoidance_move(float vx, float vy, float wz)
{
}

void Master::combine_road_obstacle_pcl()
{
}

void Master::combine_road_obstacle_pcl(int8_t *selected_lane_)
{
}

void Master::centerline_extractor()
{
}

void Master::buffer_obs_road_pcl()
{
}

void Master::free_road_detection()
{
}

void Master::generate_free_path_map()
{
}