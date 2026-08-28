// Navbar Animation
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    backgroundColor: "#fff",
    duration: 2000,
    loop: true,
});

// Connect to the ROS bridge WebSocket server
var ros = new ROSLIB.Ros({
    url: "ws://" + window.location.hostname + ":9090",
});

ros.on("connection", function () {
    console.log("Connected to WebSocket server.");
});

ros.on("error", function (error) {
    console.log("Error connecting to WebSocket server:", error);
});

ros.on("close", function () {
    console.log("Connection to WebSocket server closed.");
});


const topic_velocity_and_steering = new ROSLIB.Topic({
    ros: ros,
    name: '/master/ui_target_velocity_and_steering',
    messageType: 'std_msgs/Float32MultiArray'
});


let ui_target_velocity = 0;
let ui_target_steering = 0;

let speed_linier = 0;
let lane_used = 0;

let is_race = 0;

document.addEventListener('keydown', function (event) {

    if (event.key == 'w') {
        ui_target_velocity += 0.1;
    }
    else if (event.key == 's') {
        ui_target_velocity -= 0.2;
    }
    else if (event.key == "j") {
        ui_target_velocity = 2.5;
    }
    else if (event.key == "g") {
        ui_target_velocity = -1;
    }
    else if (event.key == 'm') {
        // ui_target_steering += -0.076928521 * 1 / 10;
        ui_target_steering -= 0.08726653; // -5 degrees in radians
    }
    else if (event.key == 'n') {
        // ui_target_steering = 0;
        ui_target_steering = 0.0;

    }
    else if (event.key == 'b') {
        // ui_target_steering += 0.076928521 * 1 / 10;
        ui_target_steering += 0.08726653;
    }
    else if (event.key == ' ') {
        ui_target_velocity = -0.0;
        ui_target_steering = 0.0;
    }
    else if (event.key == 'c') {
        ui_target_velocity = -0.0;
        ui_target_steering = 0.0;
    }


    topic_velocity_and_steering.publish({ data: [ui_target_velocity, ui_target_steering] });

});

let target_fsm_selected = 0;

// const buttonMode4 = document.getElementById('mode-4');
// buttonMode4.addEventListener('keydown', function (event) {
//     if (event.key === ' ') {
//         console.log("Button Mode 4 pressed");
//         set_master_fsm(0);
//     }
// });







//? =========================================================
//?             SLIDER
//? =========================================================

const slidersContainer = document.getElementById("sliders-container");
const sliderInputs = []; // Untuk menyimpan referensi DOM dari semua slider
const sliderName = [
    'min a1',
    'min a2',
    'min a3',
    'max a1',
    'max a2',
    'max a3',
    'min b1',
    'min b2',
    'min b3',
    'max b1',
    'max b2',
    'max b3',
    'lookahsead far',
    'lookahead near',
    '',
    '',
    '',
    '',
    '',
]
const sliderNum = 19;

// Create a ROSLIB Topic to subscribe to the 'ui_test' topic
var listener = new ROSLIB.Topic({
    ros: ros,
    name: "/vision/controlbox",
    messageType: "std_msgs/Int16MultiArray",
});

const sliderArrayTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/web/slider',
    messageType: 'std_msgs/Int16MultiArray'
});

const initialOpen = new ROSLIB.Topic({
    ros: ros,
    name: '/web/initial',
    messageType: 'std_msgs/Int8'
});


// wait 5 seconds before sending the initial open message
setTimeout(() => {
    console.log("Publishing initial open message to UI after 5 seconds");
    initialOpen.publish({ data: 1 }); // Send an initial message to indicate the UI is open
}, 800);


for (let i = 1; i <= sliderNum; i++) {
    const field = document.createElement("div");
    field.className = "field mb-2";

    const control = document.createElement("div");
    control.className = "control is-flex is-align-items-center";
    control.style.gap = "1rem";

    const label = document.createElement("span");
    label.className = "has-text-weight-semibold";
    label.style.width = "80px";
    label.textContent = sliderName[i - 1]; // Use sliderName or default text

    const input = document.createElement("input");
    input.className = "slider";
    input.type = "range";
    input.min = "0";
    input.max = "255";
    input.value = "0";
    input.style = "flex: 1";

    const valueTag = document.createElement("span");
    valueTag.className = "tag is-info";
    valueTag.textContent = "0";

    control.appendChild(label);
    control.appendChild(input);
    control.appendChild(valueTag);
    field.appendChild(control);
    slidersContainer.appendChild(field);

    sliderInputs.push(input);

    input.addEventListener("input", () => {
        valueTag.textContent = input.value;
        const allValues = sliderInputs.map(sl => parseInt(sl.value));
        const msg = new ROSLIB.Message({ data: allValues });
        console.log("Publishing slider values:", allValues);
        sliderArrayTopic.publish(msg);
    });
}

let receivedInitial = false;

listener.subscribe(function (message) {
    console.log("Received message:", message.data);

    const values = message.data;  // langsung gunakan array
    if (values.length !== sliderNum) {
        console.error(`Expected ${sliderNum} values, but received ${values.length}.`);
        return;
    }

    values.forEach((value, index) => {
        if (index < sliderInputs.length) {
            sliderInputs[index].value = value;
            sliderInputs[index].dispatchEvent(new Event('input')); // update tampilan
        }
    });
});


// send inituial publisher for initial value





//? =========================================================
//?             IMAGE
//? =========================================================

function set_video_source() {

    const stdout1 = document.getElementById('stdout1');
    stdout1.src = "http://" + window.location.hostname + ":8080/stream?topic=" + "/vision/color_depth_overlay" + "&quality=50";
    stdout1.alt = "MJPEG Stream";

    const stdout2 = document.getElementById('stdout2');
    stdout2.src = "http://" + window.location.hostname + ":8080/stream?topic=" + "/vision/debug_binary2" + "&quality=10";
    stdout2.alt = "MJPEG Stream";

    const stdout3 = document.getElementById('stdout3');
    stdout3.src = "http://" + window.location.hostname + ":8080/stream?topic=" + "/vision/debug_binary3" + "&quality=10";
    stdout3.alt = "MJPEG Stream";

    const stdout4 = document.getElementById('stdout4');
    stdout4.src = "http://" + window.location.hostname + ":8080/stream?topic=" + "/vision/filtered_binary" + "&quality=10";
    stdout4.alt = "MJPEG Stream";

    if (is_race) {
        const stdout5 = document.getElementById('stdout5');
        stdout5.src = "http://" + window.location.hostname + ":8080/stream?topic=" + "/vision/road_binary" + "&quality=10";
        stdout5.alt = "MJPEG Stream";

        const stdout6 = document.getElementById('stdout6');
        stdout6.src = "http://" + window.location.hostname + ":8080/stream?topic=" + "/vision/debug_binary" + "&quality=50";
        stdout6.alt = "MJPEG Stream";
    } else {
        const stdout5 = document.getElementById('stdout5');
        stdout5.src = "http://" + window.location.hostname + ":8080/stream?topic=" + "/vision/road_binary" + "&quality=10";
        stdout5.alt = "MJPEG Stream";

        const stdout6 = document.getElementById('stdout6');
        stdout6.src = "http://" + window.location.hostname + ":8080/stream?topic=" + "/sign/image" + "&quality=10";
        stdout6.alt = "MJPEG Stream";
    }
}

//? =========================================================
//?             INFO
//? =========================================================

const robot_vel_info = new ROSLIB.Topic({
    ros: ros,
    name: '/master/target_speed',
    // name: '/odom',
    messageType: 'std_msgs/Float32'
});


robot_vel_info.subscribe(function (message) {
    const data = message.data;
    document.getElementById('info-velocity').textContent = data.toFixed(2);

});


const robot_steering = new ROSLIB.Topic({
    ros: ros,
    name: '/master/target_steering',
    messageType: 'std_msgs/Float32'
});

robot_steering.subscribe(function (message) {
    const data = message.data;
    document.getElementById('info-steering').textContent = data.toFixed(2);
});

const robot_gyro = new ROSLIB.Topic({
    ros: ros,
    name: '/master/curr_gyro_deg',
    messageType: 'std_msgs/Float32'
});

robot_gyro.subscribe(function (message) {
    const data = message.data;
    document.getElementById('info-gyro').textContent = data.toFixed(2);
});


//? ========================================================
//?             CONFIGURATION
//? ========================================================

// Topic untuk publish konfigurasi (Float32MultiArray)
const topic_configuration = new ROSLIB.Topic({
    ros: ros,
    name: '/web/vision/configuration',
    messageType: 'std_msgs/Float32MultiArray'
});

name_configuration = [
    "kp_steering",
    "ki_steering",
    "kd_steering",
    "lookahead_far_meter",
    "lookahead_near_meter",
    "meter_to_pixel",
    "wheelbase",
    "max_steering_deg",
    "line_length_min",
    "line_length_max",
    "line_length_edge_min",
    "line_length_edge_max",
    "speed_straight",
    "speed_wiggle",
    "speed_curve",
    "lookahead_straight_distance",
    "lookahead_wiggle_distance",
    "lookahead_curve_distance",
    "used_lane",
    "valid_center_left",
    "valid_center_right",
    "valid_up",
    "valid_down",
    "cropping_distance",
    "record_vision",
    "jarak_hindar_meter",
    "out_duration_belokan",
    "out_duration_normal",
    "offset_out_duration_center",
    "max_enc_meter_obs",
    "max_enc_meter_obs_center",
    "min_dist_jarak_hindar",
    "min_dist_jarak_keluar",
    "constant_speed_belok",
    "dashed_filter_area_",
    "segment_speed_1",
    "segment_speed_2",
    "segment_speed_3",
    "segment_speed_4",
    "road_segment_threshold_area",
    "constant_transient_speed",
    "scaller_speed",
    "offset_kiri",
    "dash_kiri_default",
    "dash_kanan_default",
    "dash_kiri_anomali",
    "dash_kanan_anomali",
    "edge_kiri_default",
    "edge_kanan_default",
    "edge_kiri_anomali",
    "edge_kanan_anomali",

];

name_configuration_urban = [
    "kp_steering",
    "ki_steering",
    "kd_steering",

    "lookahead_far_meter",
    "lookahead_near_meter",
    "meter_to_pixel",
    "max_steering_deg",
    "wheelbase",
    "valid_center_left",
    "valid_center_right",
    "valid_up",
    "valid_down",
    "cropping_distance",


    "derajat_steering_kanan",
    "derajat_steering_kiri",
    "encoder_belok_kanan",
    "encoder_belok_kiri",
    "encoder_maju_kanan",
    "encoder_maju_kiri",
    "encoder_maju_lurus",
    "jarak_ke_putih",
    "derajat_gyro_kanan",
    "derajat_gyro_kiri",

    "length_titik_putih",
    "length_titik_hitam",

    "jarak_ke_zebracros_",
    "min_jarak_putih_kanan_",
    "min_jarak_putih_kiri_",
    "min_jarak_putih_lurus_",
    "encoder_maju_dead_end",
    "velocity_otomatis",

    "line_length_kanan",
    "line_length_kiri",
    "offset_jarak_sign_pole_",
    "last_gyro_angle_",
    "cntr_jalan_lurus", // Tambahkan cntr_jalan_lurus ke konfigurasi urban
    "min_vel_belokan",
    "jarak_ke_sign_pole_",

]



// Topic subscribe untuk menerima konfigurasi awal (misal dari node ROS)
const config_listener = new ROSLIB.Topic({
    ros: ros,
    name: '/vision/web/configuration_init',
    messageType: 'std_msgs/Float32MultiArray'
});

// Ambil input element per config
function getConfigInputs() {
    let inputs = [];

    if (is_race == 0) {
        name_configuration = name_configuration_urban;
    }

    for (let i = 1; i <= name_configuration.length; i++) {

        labelId = document.getElementById(`label-config-${i}`);
        labelId.innerText = name_configuration[i - 1]; // Set label text

        let el = document.getElementById(`config-${i}`);
        if (el) inputs.push(el);

    }
    return inputs;
}

// Publish konfigurasi saat "Apply" ditekan
function on_save_configuration() {
    let configArr = [];
    let inputs = getConfigInputs();
    inputs.forEach((input, idx) => {
        let val = parseFloat(input.value);
        if (isNaN(val)) val = 0;
        configArr[idx] = val;
    });
    topic_configuration.publish({ data: configArr });
    console.log("Published configuration:", configArr);
}

// Reset ke nol saat "Reset" ditekan
function on_reset_config() {

}

// Update input dari data yang masuk dari ROS
config_listener.subscribe(function (msg) {
    let data = msg.data;

    if (msg.data[0] < 50) {
        is_race = 1;
        document.getElementById('mode-now').textContent = "RACE";
    } else {
        is_race = 0;
        document.getElementById('mode-now').textContent = "URBAN";
    }

    set_video_source();


    let inputs = getConfigInputs();
    console.log("Received configuration from ROS:", msg.data[0], is_race);

    // Skip first data element (identifier), start from index 1
    data.slice(1).forEach((val, idx) => {
        if (inputs[idx]) inputs[idx].value = val;
        console.log(`Config ${idx} (${name_configuration[idx]}):`, val);
    });
    console.log("Received configuration from ROS:", data);
});


//? ========================================================
//?             STATE URBAN
//? ========================================================

const topic_state_urban = new ROSLIB.Topic({
    ros: ros,
    name: '/master/state_urban',
    messageType: 'std_msgs/Int16'
});

// callback function untuk menerima pesan dari topik
topic_state_urban.subscribe(function (message) {
    const data = message.data;
    console.log("Received state_urban:", data);
    document.getElementById('info-fsm-urban').textContent = data;
});


//? ========================================================
//?             BUTTONS
//? ========================================================

const topic_fsm = new ROSLIB.Topic({
    ros: ros,
    name: '/master/set_master_fsm',
    messageType: 'std_msgs/Int16'
});

function final_set_fsm() {
    console.log("Setting master FSM to:", target_fsm_selected);
    document.getElementById('master_fsm').textContent = "Mode: " + target_fsm_selected;
    topic_fsm.publish({ data: target_fsm_selected });
}

function set_master_fsm(target_fsm) {
    console.log("set_master_fsm called with target_fsm:", target_fsm, " and target_fsm_selected:", target_fsm_selected);
    if ((target_fsm_selected === target_fsm) && (target_fsm === 4)) {
        target_fsm_selected = 0;
    } else {
        target_fsm_selected = target_fsm;
    }

    final_set_fsm();
}

set_lane();

function set_lane() {
    // publish the selected lane to the ROS topic

    if (lane_used) {
        lane_used = 0;
    } else {
        lane_used = 1; // Toggle between 0 and 1
    }
    console.log("Setting used lane to:", lane_used);

    const usedLaneTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/web/used_lane',
        messageType: 'std_msgs/Int16'
    });

    const usedThresholdTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/web/used_threshold',
        messageType: 'std_msgs/Int16'
    });

    usedLaneTopic.publish({ data: lane_used }); // Publish 1 for right lane, 0 for left lane
    usedThresholdTopic.publish({ data: lane_used }); // Publish 0 for default threshold

    if (is_race) {
        document.getElementById('info-lane').textContent = lane_used ? "RIGHT" : "LEFT";
        document.getElementById('toggle_lane').textContent = "TOGGLE LANE";

    }
    else {
        document.getElementById('info-lane').textContent = lane_used ? "ADAPTIVE" : "FIXED";
        document.getElementById('toggle_lane').textContent = "TOGGLE THRES";
    }

}

let toggle_debug = 2;
let toggle_debug2 = 4;

set_debug();
set_debug2();

function set_debug() {
    // publish the selected lane to the ROS topic

    if (toggle_debug == 0) {
        toggle_debug = 1;
        document.getElementById('info-debug').textContent = "DEBUG: ENC";
    } else if (toggle_debug == 1) {
        toggle_debug = 2;
        document.getElementById('info-debug').textContent = "DEBUG: GYRO";
    } else if (toggle_debug == 2) {
        toggle_debug = 0;
        document.getElementById('info-debug').textContent = "DEBUG: NONE";
    }

    console.log("Setting used lane to:", toggle_debug);

    const toggle_debug_topic = new ROSLIB.Topic({
        ros: ros,
        name: '/web/toggle_debug',
        messageType: 'std_msgs/Int16'
    });

    toggle_debug_topic.publish({ data: toggle_debug }); // Publish 1 for right lane, 0 for left lane
}

function set_debug2() {
    // publish the selected lane to the ROS topic

    if (toggle_debug2 == 0) {
        toggle_debug2 = 1;
        document.getElementById('info-debug2').textContent = "DEBUG: LEFT";
    } else if (toggle_debug2 == 1) {
        toggle_debug2 = 2;
        document.getElementById('info-debug2').textContent = "DEBUG: RIGHT";
    } else if (toggle_debug2 == 2) {
        toggle_debug2 = 3;
        document.getElementById('info-debug2').textContent = "DEBUG: FORWARD";
    } else if (toggle_debug2 == 3) {
        toggle_debug2 = 4;
        document.getElementById('info-debug2').textContent = "DEBUG: DEFAULT";
    } else if (toggle_debug2 == 4) {
        toggle_debug2 = 0;
        document.getElementById('info-debug2').textContent = "AUTO";
    }

    console.log("Setting used lane to:", toggle_debug2);

    const toggle_debug2_topic = new ROSLIB.Topic({
        ros: ros,
        name: '/web/toggle_debug2',
        messageType: 'std_msgs/Int16'
    });

    toggle_debug2_topic.publish({ data: toggle_debug2 }); // Publish 1 for right lane, 0 for left lane
}
