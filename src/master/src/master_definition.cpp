#include "master/master.hpp"

void Master::process_transmitter()
{
    std_msgs::msg::Float32 msg_target_velocity;
    msg_target_velocity.data = actuation_vx;
    pub_target_velocity->publish(msg_target_velocity);

    std_msgs::msg::Float32 msg_target_steering;
    msg_target_steering.data = actuation_wz;
    pub_target_Steering->publish(msg_target_steering);

    std_msgs::msg::Int16 msg_global_fsm;
    msg_global_fsm.data = global_fsm.value;
    pub_global_fsm->publish(msg_global_fsm);

    std_msgs::msg::Int16 msg_local_fsm;
    msg_local_fsm.data = local_fsm.value;
    pub_local_fsm->publish(msg_local_fsm);

    static uint16_t divider_waypoint_pub_counter = 0;
    if (divider_waypoint_pub_counter++ >= 25)
    {
        divider_waypoint_pub_counter = 0;

        sensor_msgs::msg::PointCloud msg_waypoints;
        for (auto i : waypoints)
        {
            geometry_msgs::msg::Point32 p;
            p.x = i.x;
            p.y = i.y;
            p.z = 0;
            msg_waypoints.points.push_back(p);
        }
        pub_waypoints->publish(msg_waypoints);

        //=================================================

        sensor_msgs::msg::PointCloud msg_waypoints_kanan;
        for (auto i : waypoints_race_kanan)
        {
            geometry_msgs::msg::Point32 p;
            p.x = i.x;
            p.y = i.y;
            p.z = 0;
            msg_waypoints_kanan.points.push_back(p);
        }
        pub_waypoints_kanan->publish(msg_waypoints_kanan);

        //=================================================

        sensor_msgs::msg::PointCloud msg_waypoints_kiri;
        for (auto i : waypoints_race_kiri)
        {
            geometry_msgs::msg::Point32 p;
            p.x = i.x;
            p.y = i.y;
            p.z = 0;
            msg_waypoints_kiri.points.push_back(p);
        }
        pub_waypoints_kiri->publish(msg_waypoints_kiri);

        //=================================================

        sensor_msgs::msg::PointCloud msg_waypoints_tengah;
        for (auto i : waypoints_race_tengah)
        {
            geometry_msgs::msg::Point32 p;
            p.x = i.x;
            p.y = i.y;
            p.z = 0;
            msg_waypoints_tengah.points.push_back(p);
        }
        pub_waypoints_tengah->publish(msg_waypoints_tengah);

        pub_terminals->publish(terminals);
    }
}

void Master::process_load_waypoints()
{
    waypoints.clear();

    if (!boost::filesystem::exists(waypoint_file_path))
    {
        logger.error("File %s does not exist. Create a new file.", waypoint_file_path.c_str());
        std::ofstream file(waypoint_file_path);
        file << "x,y,fb_velocity,fb_steering" << std::endl;
        file.close();
    }

    std::ifstream file;

    file.open(waypoint_file_path);
    if (file.is_open())
    {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        while (file.good())
        {
            if (file.peek() == EOF)
                break;
            waypoint_t wp;
            file >> wp.x;
            file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file >> wp.y;
            file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file >> wp.fb_velocity;
            file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file >> wp.fb_steering;
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            if (transform_map2odom)
            {
                tf2::Transform tf_wp;
                tf_wp.setOrigin(tf2::Vector3(wp.x, wp.y, 0));
                tf2::Quaternion q;
                q.setRPY(0, 0, 0);
                tf_wp.setRotation(q);

                tf2::Transform tf_transformed = manual_map2odom_tf * tf_wp;
                wp.x = tf_transformed.getOrigin().getX();
                wp.y = tf_transformed.getOrigin().getY();
            }

            waypoints.push_back(wp);
        }
        file.close();

        for (size_t i = 0; i + 1 < waypoints.size(); ++i)
        {
            float dx = waypoints[i + 1].x - waypoints[i].x;
            float dy = waypoints[i + 1].y - waypoints[i].y;
            waypoints[i].arah = std::atan2(dy, dx);
        }

        // Asumsikan heading waypoint terakhir sama dengan sebelumnya
        if (waypoints.size() >= 2)
            waypoints.back().arah = waypoints[waypoints.size() - 2].arah;
        else if (waypoints.size() == 1)
            waypoints.back().arah = 0.0; // default jika hanya 1 waypoint

        logger.info("Read %d waypoints from file %s.", waypoints.size(), waypoint_file_path.c_str());
    }
}

void Master::process_load_waypoints_race(std::string file_path, std::vector<waypoint_t> &wps)
{
    wps.clear();

    if (!boost::filesystem::exists(file_path))
    {
        logger.error("File %s does not exist. Create a new file.", file_path.c_str());
        std::ofstream file(file_path);
        file << "x,y,fb_velocity,fb_steering" << std::endl;
        file.close();
    }

    std::ifstream file;

    file.open(file_path);
    if (file.is_open())
    {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        while (file.good())
        {
            if (file.peek() == EOF)
                break;
            waypoint_t wp;
            file >> wp.x;
            file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file >> wp.y;
            file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file >> wp.fb_velocity;
            file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file >> wp.fb_steering;
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            if (transform_map2odom)
            {
                tf2::Transform tf_wp;
                tf_wp.setOrigin(tf2::Vector3(wp.x, wp.y, 0));
                tf2::Quaternion q;
                q.setRPY(0, 0, 0);
                tf_wp.setRotation(q);

                tf2::Transform tf_transformed = manual_map2odom_tf * tf_wp;
                wp.x = tf_transformed.getOrigin().getX();
                wp.y = tf_transformed.getOrigin().getY();
            }

            wps.push_back(wp);
        }
        file.close();

        for (size_t i = 0; i + 1 < wps.size(); ++i)
        {
            float dx = wps[i + 1].x - wps[i].x;
            float dy = wps[i + 1].y - wps[i].y;
            wps[i].arah = std::atan2(dy, dx);
        }

        // Asumsikan heading waypoint terakhir sama dengan sebelumnya
        if (wps.size() >= 2)
            wps.back().arah = wps[wps.size() - 2].arah;
        else if (wps.size() == 1)
            wps.back().arah = 0.0; // default jika hanya 1 waypoint

        logger.info("Read %d wps from file %s.", wps.size(), file_path.c_str());
    }
}

void Master::process_save_waypoints()
{
    std::ofstream file;
    file.open(waypoint_file_path);

    if (file.is_open())
    {
        file << "x,y,fb_velocity,fb_steering" << std::endl;
        for (auto i : waypoints)
            file << i.x << "," << i.y << "," << i.fb_velocity << "," << i.fb_steering << std::endl;
        file.close();
        logger.info("Saved %d waypoints to file %s.", waypoints.size(), waypoint_file_path.c_str());
    }
    else
    {
        logger.error("Failed to save waypoints to file %s.", waypoint_file_path.c_str());
    }
}

void Master::process_save_waypoints_race(std::string file_path, std::vector<waypoint_t> &wps)
{
    std::ofstream file;
    file.open(file_path);

    if (file.is_open())
    {
        file << "x,y,fb_velocity,fb_steering" << std::endl;
        for (auto i : wps)
            file << i.x << "," << i.y << "," << i.fb_velocity << "," << i.fb_steering << std::endl;
        file.close();
        logger.info("Saved %d wps to file %s.", wps.size(), file_path.c_str());
    }
    else
    {
        logger.error("Failed to save wps to file %s.", file_path.c_str());
    }
}

void Master::process_record_route()
{
    static float prev_x = fb_final_pose_xyo[0];
    static float prev_y = fb_final_pose_xyo[1];

    float dx = fb_final_pose_xyo[0] - prev_x;
    float dy = fb_final_pose_xyo[1] - prev_y;
    float d = sqrt(dx * dx + dy * dy);

    if (d > 0.02)
    {
        waypoint_t wp;
        wp.x = fb_final_pose_xyo[0];
        wp.y = fb_final_pose_xyo[1];
        wp.fb_velocity = fb_encoder_meter;
        wp.fb_steering = fb_steering_angle;
        waypoints.push_back(wp);
        prev_x = wp.x;
        prev_y = wp.y;
        logger.info("Recorded waypoint %.2f %.2f %.2f %.2f", wp.x, wp.y, wp.fb_velocity, wp.fb_steering);
    }
}

void Master::process_record_route(std::vector<waypoint_t> &wps)
{
    static float prev_x = fb_final_pose_xyo[0];
    static float prev_y = fb_final_pose_xyo[1];

    float dx = fb_final_pose_xyo[0] - prev_x;
    float dy = fb_final_pose_xyo[1] - prev_y;
    float d = sqrt(dx * dx + dy * dy);

    if (d > 0.02)
    {
        waypoint_t wp;
        wp.x = fb_final_pose_xyo[0];
        wp.y = fb_final_pose_xyo[1];
        wp.fb_velocity = fb_encoder_meter;
        wp.fb_steering = fb_steering_angle;
        wps.push_back(wp);
        prev_x = wp.x;
        prev_y = wp.y;
        logger.info("Recorded waypoint %.2f %.2f %.2f %.2f", wp.x, wp.y, wp.fb_velocity, wp.fb_steering);
    }
}

void Master::process_load_terminals()
{
    std::ifstream fin;
    std::string line;
    std::vector<std::string> tokens;
    bool is_terminal_loaded_normally = false;
    try
    {
        fin.open(terminal_file_path, std::ios::in);
        if (fin.is_open())
        {
            is_terminal_loaded_normally = true;
            while (std::getline(fin, line))
            {
                if (line.find("type") != std::string::npos)
                    continue;
                boost::split(tokens, line, boost::is_any_of(","));

                for (auto &token : tokens)
                    boost::trim(token);

                ros2_interface::msg::Terminal terminal;
                terminal.type = std::stoi(tokens[0]);
                terminal.id = std::stoi(tokens[1]);
                terminal.target_pose_x = std::stof(tokens[2]);
                terminal.target_pose_y = std::stof(tokens[3]);
                terminal.target_pose_theta = std::stof(tokens[4]);
                terminal.target_max_velocity_x = std::stof(tokens[5]);
                terminal.target_max_velocity_y = std::stof(tokens[6]);
                terminal.target_max_velocity_theta = std::stof(tokens[7]);
                terminal.radius_area = std::stof(tokens[8]);
                terminal.target_lookahead_distance = std::stof(tokens[9]);
                terminal.obs_scan_r = std::stof(tokens[10]);
                terminal.stop_time_s = std::stof(tokens[11]);
                terminal.scan_min_x = std::stof(tokens[12]);
                terminal.scan_max_x = std::stof(tokens[13]);
                terminal.scan_min_y = std::stof(tokens[14]);
                terminal.scan_max_y = std::stof(tokens[15]);
                terminal.obs_threshold = std::stof(tokens[16]);

                if (transform_map2odom)
                {
                    tf2::Transform tf_terminal_pose;
                    tf_terminal_pose.setOrigin(tf2::Vector3(terminal.target_pose_x, terminal.target_pose_y, 0));
                    tf2::Quaternion q;
                    q.setRPY(0, 0, terminal.target_pose_theta);
                    tf_terminal_pose.setRotation(q);

                    tf2::Transform tf_transformed = manual_map2odom_tf * tf_terminal_pose;

                    terminal.target_pose_x = tf_transformed.getOrigin().getX();
                    terminal.target_pose_y = tf_transformed.getOrigin().getY();

                    tf2::Matrix3x3 m(tf_transformed.getRotation());
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);
                    terminal.target_pose_theta = yaw;
                }

                terminals.terminals.push_back(terminal);

                tokens.clear();
            }
            fin.close();

            logger.info("Terminal file loaded");
        }

        if (fin.fail() && !fin.is_open() && !is_terminal_loaded_normally)
        {
            logger.warn("Failed to load terminal file, Recreate the terminal file");
            process_save_terminals();
        }
    }
    catch (const std::exception &e)
    {
        logger.error("Failed to load terminal file: %s, Recreate the terminal file", e.what());
        process_save_terminals();
    }
}

void Master::process_save_terminals()
{
    std::ofstream fout;

    try
    {
        fout.open(terminal_file_path, std::ios::out);
        if (fout.is_open())
        {
            fout << "type, id, x, y, theta, max_vx, max_vy, max_vtheta, radius_area, lookahead_distance, obs_scan_r, stop_time_s, scan_min_x, scan_max_x, scan_min_y, scan_max_y, obs_threshold" << std::endl;
            for (auto terminal : terminals.terminals)
            {
                int terminal_type_integer = terminal.type;
                int terminal_id_integer = terminal.id;
                fout << terminal_type_integer << ", " << terminal_id_integer << ", " << terminal.target_pose_x << ", " << terminal.target_pose_y << ", " << terminal.target_pose_theta << ", " << terminal.target_max_velocity_x << ", " << terminal.target_max_velocity_y << ", " << terminal.target_max_velocity_theta << ", " << terminal.radius_area << ", " << terminal.target_lookahead_distance << ", " << terminal.obs_scan_r << ", " << terminal.stop_time_s << ", " << terminal.scan_min_x << ", " << terminal.scan_max_x << ", " << terminal.scan_min_y << ", " << terminal.scan_max_y << ", " << terminal.obs_threshold << std::endl;
            }
            fout.close();

            logger.info("Terminal file saved");
        }
    }
    catch (const std::exception &e)
    {
        logger.warn("Failed to open terminal file, Recreate the terminal file");
    }
}

void Master::process_add_terminal()
{
    static float prev_x = -9999;
    static float prev_y = -9999;

    float dx = fb_final_pose_xyo[0] - prev_x;
    float dy = fb_final_pose_xyo[1] - prev_y;
    float d = sqrt(dx * dx + dy * dy);

    /* Prevent double klik atau semacamnya */
    if (d > 0.2)
    {
        ros2_interface::msg::Terminal terminal;
        terminal.type = TERMINAL_TYPE_STOP;
        terminal.id = terminals.terminals.size();
        terminal.target_pose_x = fb_final_pose_xyo[0];
        terminal.target_pose_y = fb_final_pose_xyo[1];
        terminal.target_pose_theta = fb_final_pose_xyo[2];
        terminal.target_max_velocity_x = 0.9;
        terminal.target_max_velocity_y = 0.9;
        terminal.target_max_velocity_theta = 0.2; // gk dipakai
        terminal.radius_area = 0.5;
        terminal.target_lookahead_distance = 1;
        terminal.obs_scan_r = 2;
        terminal.stop_time_s = 10;
        terminals.terminals.push_back(terminal);
        logger.info("Add Terminal Success");
    }
}

void Master::process_add_terminal_sign()
{
    static float prev_x = -9999;
    static float prev_y = -9999;

    float dx = fb_final_pose_xyo[0] - prev_x;
    float dy = fb_final_pose_xyo[1] - prev_y;
    float d = sqrt(dx * dx + dy * dy);

    /* Prevent double klik atau semacamnya */
    if (d > 0.2)
    {
        ros2_interface::msg::Terminal terminal;
        terminal.type = TERMINAL_TYPE_STOP1;
        terminal.id = terminals.terminals.size();
        terminal.target_pose_x = fb_final_pose_xyo[0];
        terminal.target_pose_y = fb_final_pose_xyo[1];
        terminal.target_pose_theta = fb_final_pose_xyo[2];
        terminal.target_max_velocity_x = 0.0;
        terminal.target_max_velocity_y = 0.9;
        terminal.target_max_velocity_theta = 0.2; // gk dipakai
        terminal.radius_area = 0.4;
        terminal.target_lookahead_distance = 0.6;
        terminal.obs_scan_r = 2;
        terminal.stop_time_s = 10;
        terminals.terminals.push_back(terminal);
        logger.info("Add Terminal Success");
    }
}