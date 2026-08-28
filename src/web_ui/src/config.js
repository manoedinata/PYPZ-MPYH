// Connect ke ROS bridge websocket
var ros = new ROSLIB.Ros({
    url: "ws://" + window.location.hostname + ":9090",
});

// Topic untuk publish konfigurasi (Float32MultiArray)
const topic_configuration = new ROSLIB.Topic({
    ros: ros,
    name: '/web/configuration',
    messageType: 'std_msgs/Float32MultiArray'
});

// Topic subscribe untuk menerima konfigurasi awal (misal dari node ROS)
const config_listener = new ROSLIB.Topic({
    ros: ros,
    name: '/web/configuration',
    messageType: 'std_msgs/Float32MultiArray'
});

// Ambil input element per config
function getConfigInputs() {
    let inputs = [];
    for (let i = 1; i <= 8; i++) {
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
    // Buat publisher ke '/web/request_config' kalau kamu pakai handshake manual
    // var reqConfig = new ROSLIB.Topic({
    //     ros: ros,
    //     name: '/web/request_config',
    //     messageType: 'std_msgs/Empty'
    // });
    // reqConfig.publish({});
};
