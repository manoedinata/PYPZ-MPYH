// Connect ke ROS bridge websocket
var ros = new ROSLIB.Ros({
    url: "ws://" + window.location.hostname + ":9090",
});

// Topic untuk publish konfigurasi (Float32MultiArray)
const topic_configuration = new ROSLIB.Topic({
    ros: ros,
    name: '/web/config/configuration',
    messageType: 'std_msgs/Float32MultiArray'
});

name_configuration = [
    "k_p_wheel",
    "k_i_wheel",
    "k_d_wheel",
    "k_d_steering",
    "wheel_radius",
    "encoder_ppr",
    "cnt_to_meter",
    "max_steering_deg",
    "min_steering_deg",
    "max_steering_pwm",
    "min_steering_pwm",
    "mid_steering_pwm",
    "max_wheel_velocity_pwm",
    "min_wheel_velocity_pwm",
    "max_wheel_integral_pwm",
    "min_wheel_integral_pwm",
    "wheel_base",
    "tuning", // Tambahan baru
    "K_model", // Tambahan baru//
];

// Topic subscribe untuk menerima konfigurasi awal (misal dari node ROS)
const config_listener = new ROSLIB.Topic({
    ros: ros,
    name: '/web/config/configuration_init',
    messageType: 'std_msgs/Float32MultiArray'
});

// Ambil input element per config
function getConfigInputs() {
    let inputs = [];
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
    let inputs = getConfigInputs();
    inputs.forEach(input => input.value = 0);
    // (Optional) Kirim array reset juga ke ROS:
    let zeros = new Array(8).fill(0);
    topic_configuration.publish({ data: zeros });
    console.log("Reset configuration to zeros");
}

// Update input dari data yang masuk dari ROS
config_listener.subscribe(function (msg) {
    let data = msg.data;
    let inputs = getConfigInputs();
    data.forEach((val, idx) => {
        if (inputs[idx]) inputs[idx].value = val;
    });
    console.log("Received configuration from ROS:", data);
});

// (Optional) Initial request ke ROS agar node mengirim config awal
window.onload = function () {
    // Buat publisher ke '/web/config/request_config' kalau kamu pakai handshake manual
    var reqConfig = new ROSLIB.Topic({
        ros: ros,
        name: '/web/config/request_config',
        messageType: 'std_msgs/Int16'
    });
    reqConfig.publish({});
};

const topic_velocity_and_steering = new ROSLIB.Topic({
    ros: ros,
    name: '/master/ui_target_velocity_and_steering',
    messageType: 'std_msgs/Float32MultiArray'
});

const topic_pwm = new ROSLIB.Topic({
    ros: ros,
    name: '/master/target_speed',
    messageType: 'std_msgs/Float32'
});


let ui_target_velocity = 0;
let ui_target_steering = 0;

let pwm_motor = 0;

let speed_linier = 0;

document.addEventListener('keydown', function (event) {

    if (event.key == 'w') {
        ui_target_velocity += 0.1;
    }
    else if (event.key == 's') {
        ui_target_velocity -= 0.1;
    }
    else if (event.key == "j") {
        ui_target_velocity = 2.0;
    }
    else if (event.key == "g") {
        ui_target_velocity = -1;
    }
    else if (event.key == 'm') {
        // ui_target_steering += -0.076928521 * 1 / 10;
        ui_target_steering -= 0.1;
    }
    else if (event.key == 'n') {
        // ui_target_steering = 0;
        ui_target_steering = 0.0;

    }
    else if (event.key == 'b') {
        // ui_target_steering += 0.076928521 * 1 / 10;
        ui_target_steering += 0.1;
    }
    else if (event.key == ' ') {
        ui_target_velocity = -0.0;
        ui_target_steering = 0.0;

        pwm_motor = 0.0; // Stop
    }
    // if i hit 'enter' key, publish the velocity and steering
    else if (event.key == 'Enter') {
        on_save_configuration();
    }
    else if (event.key == '1') {
        pwm_motor = 0; // Stop
    }
    else if (event.key == '2') {
        pwm_motor = 1000; // Stop
    }
    else if (event.key == '3') {
        pwm_motor = 2000; // Stop
    }
    else if (event.key == '4') {
        pwm_motor = 3000; // Stop
    }
    else if (event.key == '5') {
        pwm_motor = 4000; // Stop
    }


    topic_velocity_and_steering.publish({ data: [ui_target_velocity, ui_target_steering] });
    // topic_pwm.publish({ data: pwm_motor });
});


//? ========================================================
//?             CHART
//? ========================================================

// Chart.js setup
const dataWindow = 1000; // How many points to show

const ctxVel = document.getElementById('rosChartVelocity').getContext('2d');
const rosChartVelocity = new Chart(ctxVel, {
    type: 'line',
    data: {
        labels: Array(dataWindow).fill(''),
        datasets: [{
            label: 'Velocity (m/s)',
            data: Array(dataWindow).fill(0),
            fill: false,
            borderColor: 'rgb(52,152,219)',
            tension: 0.1
        }]
    },
    options: {
        animation: false,
        scales: {
            x: { display: true, title: { display: true, text: 't' } },
            y: {
                min: -0.5, max: 1.5,
                ticks: {
                    font: { size: 18 } // Make X-axis numbers bigger
                },
                grid: { color: '#eee' } // light grid
            }
        }
    }
});

// STEERING CHART
const ctxSteer = document.getElementById('rosChartSteering').getContext('2d');
const rosChartSteering = new Chart(ctxSteer, {
    type: 'line',
    data: {
        labels: Array(dataWindow).fill(''),
        datasets: [{
            label: 'Velocity (deg/s)',
            data: Array(dataWindow).fill(0),
            fill: false,
            borderColor: 'rgb(219, 52, 52)',
            tension: 0.1,
        }]
    },
    options: {
        animation: false,
        scales: {
            x: { display: true, title: { display: true, text: 't' } },
            y: {
                min: -45,
                max: 45,
                ticks: {
                    font: { size: 18 } // Make X-axis numbers bigger
                },
                grid: { color: '#eee' } // light grid
            }

        }
    }
});

const t0 = Date.now(); // Record when page loaded (in ms)

const robot_vel = new ROSLIB.Topic({
    ros: ros,
    name: '/motor_main/velocity_feedback',
    messageType: 'std_msgs/Float32'
});

const robot_vel_info = new ROSLIB.Topic({
    ros: ros,
    name: '/master/target_speed',
    messageType: 'std_msgs/Float32'
});

robot_vel_info.subscribe(function (message) {
    const data = message.data;
    document.getElementById('info-velocity').textContent = data.toFixed(2);

});

robot_vel.subscribe(function (message) {
    const data = message.data;

    const tStep = ((Date.now() - t0) / 1000).toFixed(2);

    rosChartVelocity.data.labels.push(tStep);
    rosChartVelocity.data.labels.shift();

    rosChartVelocity.data.datasets[0].data.push(data);
    rosChartVelocity.data.datasets[0].data.shift();

    rosChartVelocity.update('none');
});

const robot_steering = new ROSLIB.Topic({
    ros: ros,
    name: '/master/target_steering',
    messageType: 'std_msgs/Float32'
});

robot_steering.subscribe(function (message) {
    const data = message.data;
    document.getElementById('info-steering').textContent = data.toFixed(2);

    const tStep = ((Date.now() - t0) / 1000).toFixed(2);

    rosChartSteering.data.labels.push(tStep);
    rosChartSteering.data.labels.shift();

    // Update steering chart
    rosChartSteering.data.datasets[0].data.push(data * (180 / Math.PI)); // Convert rad to deg
    rosChartSteering.data.datasets[0].data.shift();

    rosChartSteering.update('none');
});

//? ========================================================
//?             TARGET FSM
//? ========================================================
let target_fsm_selected = 0; // Default to 0


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