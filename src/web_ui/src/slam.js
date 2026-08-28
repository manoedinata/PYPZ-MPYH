class Robot {
    constructor(ip, conv_circle, conv_line, x, y, theta, radius) {
        this.ip = ip;
        this.conv_circle = conv_circle;
        this.conv_line = conv_line;
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.radius = radius;
        this.routes = [];
        this.routes_line = [];

        this.color_r = 0;
        this.color_g = 0;
        this.color_b = 0;

        this.has_finished_routes_init = false;
        this.prev_has_finished_routes_init = false;
        this.has_route_drawed = false;

    }
}

// ================================================================================================================================

let robots = [];

let posisi_robot = 0; // 0 = kiri, 1 = kanan
let posisi_obs = -1; // -1 = tidak ada, 0 = kiri, 1 = kanan

// ================================================================================================================================


let wtf_skala = 100;
// let wtf_skala = 60;


// Create a Konva Stage
const stage = new Konva.Stage({
    container: 'map',
    width: window.innerWidth,
    height: window.innerHeight,
});

// Create a layer for grid and shapes
const gridLayer = new Konva.Layer();
const robotLayer = new Konva.Layer();
const mapLayer = new Konva.Layer();
const lidarLayer = new Konva.Layer();
const waypointsLayer = new Konva.Layer();
const terminalsLayer = new Konva.Layer();
const filteredLidarLayer = new Konva.Layer();
stage.add(mapLayer);
stage.add(gridLayer);
stage.add(waypointsLayer);
stage.add(terminalsLayer);
stage.add(lidarLayer);
// stage.add(filteredLidarLayer);
stage.add(robotLayer);

// Draw the grid
const gridSize = 50;
for (let x = 0; x < stage.width(); x += gridSize) {
    gridLayer.add(
        new Konva.Line({
            points: [x, 0, x, stage.height()],
            stroke: '#ddd',
            strokeWidth: 1,
        })
    );
}
for (let y = 0; y < stage.height(); y += gridSize) {
    gridLayer.add(
        new Konva.Line({
            points: [0, y, stage.width(), y],
            stroke: '#ddd',
            strokeWidth: 1,
        })
    );
}
gridLayer.draw();

// ================================================================================================================================

// Enable zooming
stage.on('wheel', (e) => {
    e.evt.preventDefault();
    const scaleBy = 1.1;
    const oldScale = stage.scaleX();
    const pointer = stage.getPointerPosition();
    const mousePointTo = {
        x: (pointer.x - stage.x()) / oldScale,
        y: (pointer.y - stage.y()) / oldScale,
    };

    const newScale = e.evt.deltaY > 0 ? oldScale / scaleBy : oldScale * scaleBy;
    stage.scale({ x: newScale, y: newScale });

    const newPos = {
        x: pointer.x - mousePointTo.x * newScale,
        y: pointer.y - mousePointTo.y * newScale,
    };
    stage.position(newPos);
    stage.batchDraw();
});

// Enable map shifting (panning) with the right mouse button
let isDragging = false;
let dragStartPos = { x: 0, y: 0 };

stage.on('mousedown', (e) => {
    if (e.evt.button === 2) { // Check if the right mouse button is pressed
        isDragging = true;
        dragStartPos = stage.getPointerPosition();
    }

    if (e.evt.button === 1 || e.evt.button === 2) {
        isDragging = true;
        dragStartPos = stage.getPointerPosition();
    }
});

stage.on('mousemove', (e) => {
    if (!isDragging) return;

    const pointer = stage.getPointerPosition();
    const dx = pointer.x - dragStartPos.x;
    const dy = pointer.y - dragStartPos.y;

    stage.position({
        x: stage.x() + dx,
        y: stage.y() + dy,
    });
    stage.batchDraw();
    dragStartPos = pointer;
});

stage.on('mouseup', () => {
    isDragging = false;
});

stage.on('contextmenu', (e) => {
    // Prevent the browser's context menu from appearing on right-click
    e.evt.preventDefault();
});

// Handle window resizing
window.addEventListener('resize', () => {
    const width = window.innerWidth;
    const height = window.innerHeight;

    stage.width(width);
    stage.height(height);

    gridLayer.batchDraw();
    robotLayer.batchDraw();
    mapLayer.batchDraw();
    lidarLayer.batchDraw();
});

// ================================================================================================================================

function addRobot(ip, x, y, theta, radius, colourr) {
    const original_x = x;
    const original_y = y;
    const original_theta = theta;

    ip = ip;
    x = x + stage.width() * 0.5 / wtf_skala;
    y = stage.height() * 0.5 / wtf_skala - y;
    theta = theta;
    radius = radius;


    for (let i = 0; i < robots.length; i++) {
        if (robots[i].ip == ip) {
            robots[i].conv_circle.position({ x: x * wtf_skala, y: y * wtf_skala });
            robots[i].conv_line.points([x * wtf_skala, y * wtf_skala, x * wtf_skala + radius * wtf_skala * Math.cos(theta), y * wtf_skala - radius * wtf_skala * Math.sin(theta)]);
            robots[i].x = original_x;
            robots[i].y = original_y;
            robots[i].theta = original_theta;
            robots[i].radius = radius;
            robots[i].conv_circle.fill(colourr);

            robotLayer.batchDraw();

            return;
        }
    }

    const conv_circle = new Konva.Circle({
        x: x * wtf_skala,
        y: y * wtf_skala,
        radius: radius * wtf_skala,
        fill: colourr,
        draggable: true,
    });

    // Calculate the end point of the line based on theta
    const thetaRadians = theta; // Convert to radians
    const lineEndX = x * wtf_skala + radius * wtf_skala * Math.cos(thetaRadians);
    const lineEndY = y * wtf_skala - radius * wtf_skala * Math.sin(thetaRadians);

    // Draw the line inside the circle
    const conv_line = new Konva.Line({
        points: [x * wtf_skala, y * wtf_skala, lineEndX, lineEndY],
        stroke: 'Cyan',
        strokeWidth: 5,
    });

    let robot_buffer = new Robot(ip, conv_circle, conv_line, x, y, theta, radius);

    robots.push(robot_buffer);

    robotLayer.add(robot_buffer.conv_circle);
    robotLayer.add(robot_buffer.conv_line);
    robotLayer.draw();
}

// ================================================================================================================================

// Navbar Animation
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    backgroundColor: "#fff",
    duration: 2000,
    loop: true,
});

// ================================================================

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

const robotTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/slam/odometry/filtered',
    // name: '/odom',
    messageType: 'nav_msgs/Odometry'
});



robotTopic.subscribe(function (message) {
    const x = message.pose.pose.position.x;
    const y = message.pose.pose.position.y;
    const q = message.pose.pose.orientation;
    const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    const radius = 0.15;

    if (posisi_robot == 0) {
        addRobot("0.0.0.0", x, y, theta, radius, 'magenta');
    } else if (posisi_robot == 1) {
        addRobot("0.0.0.0", x, y, theta, radius, 'yellow');
    }
}
);


const obsTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/master/nearest_obstacle',
    // name: '/odom',
    messageType: 'nav_msgs/Odometry'
});

obsTopic.subscribe(function (message) {
    const x = message.pose.pose.position.x;
    const y = message.pose.pose.position.y;
    const q = message.pose.pose.orientation;
    const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    const radius = 0.15;

    if (posisi_obs == 0) {
        addRobot("0.1.0.1", x, y, theta, radius, 'red');
    } else if (posisi_obs == 1) {
        addRobot("0.1.0.1", x, y, theta, radius, 'blue');
    }
}
);

const posisi_robotTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/master/posisi_robot',
    messageType: 'std_msgs/Int8'
});
posisi_robotTopic.subscribe(function (message) {
    posisi_robot = message.data;
});

const posisi_obstacleTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/master/posisi_obstacle',
    messageType: 'std_msgs/Int8'
});
posisi_obstacleTopic.subscribe(function (message) {
    posisi_obs = message.data;
    // console.log("Posisi Robot: " + posisi_robot + ", Posisi Obstacle: " + posisi_obs);
});




// const robotTopic2 = new ROSLIB.Topic({
//     ros: ros,
//     name: '/slam/localization_pose',
//     messageType: 'geometry_msgs/PoseWithCovarianceStamped'
// });

// robotTopic2.subscribe(function (message) {
//     const x = message.pose.pose.position.x;
//     const y = message.pose.pose.position.y;
//     const q = message.pose.pose.orientation;
//     const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
//     const radius = 0.35;

//     // console.log("x: " + x + " y: " + y + " theta: " + theta);

//     addRobot("1.1.1.1", x, y, theta, radius, 'Yellow');
// }
// );

let rtabmap_current_ref_id = 0;

const slam_info = new ROSLIB.Topic({
    ros: ros,
    name: '/slam/info',
    messageType: 'rtabmap_msgs/Info'
});

slam_info.subscribe(function (message) {
    rtabmap_current_ref_id = message.ref_id;
}
);

const lidar_tf_x = 0.15;
const lidar_tf_y = 0.0;
const lidar_tf_theta = 0;

const lidarTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/vision/laserscan',
    messageType: 'sensor_msgs/LaserScan'
});


lidarTopic.subscribe(function (message) {
    // Tunggu ketika rtabmap ready
    if (robots.length == 0 || rtabmap_current_ref_id == 0) {
        return;
    }

    const ranges = message.ranges;
    const angleIncrement = message.angle_increment;
    const angleMin = message.angle_min + (robots[0].theta);

    const rotated_x = lidar_tf_x * Math.cos(robots[0].theta) - lidar_tf_y * Math.sin(robots[0].theta);
    const rotated_y = lidar_tf_x * Math.sin(robots[0].theta) + lidar_tf_y * Math.cos(robots[0].theta);

    const laserPoints = [];
    for (let i = 0; i < ranges.length; i++) {
        const angle = angleMin + i * angleIncrement;
        let x = ranges[i] * Math.cos(angle) + robots[0].x + rotated_x;
        let y = ranges[i] * Math.sin(angle) + robots[0].y + rotated_y;

        x = x + stage.width() * 0.5 / wtf_skala;
        y = stage.height() * 0.5 / wtf_skala - y;

        x = x * wtf_skala;
        y = y * wtf_skala;

        laserPoints.push(x, y);
    }

    // Clear the shape layer
    lidarLayer.destroyChildren();

    // Draw the laser scan
    const laserLine = new Konva.Line({
        points: laserPoints,
        stroke: 'red',
        strokeWidth: 1,
    });
    lidarLayer.add(laserLine);

    lidarLayer.draw();
}
);

const filteredLidarTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/obstacle_filter/filtered_lidar_pcl',
    messageType: 'sensor_msgs/PointCloud'
});

filteredLidarTopic.subscribe(function (message) {
    if (robots.length == 0) {
        return;
    }

    const points = message.points;
    const filtered_lidar = [];

    const rotated_x = lidar_tf_x * Math.cos(robots[0].theta) - lidar_tf_y * Math.sin(robots[0].theta);
    const rotated_y = lidar_tf_x * Math.sin(robots[0].theta) + lidar_tf_y * Math.cos(robots[0].theta);

    for (let i = 0; i < points.length; i++) {
        const x = points[i].x + robots[0].x + rotated_x;
        const y = -points[i].y + robots[0].y + rotated_y;

        const x_tf = x + stage.width() * 0.5 / wtf_skala;
        const y_tf = stage.height() * 0.5 / wtf_skala - y;

        filtered_lidar.push(x_tf * wtf_skala, y_tf * wtf_skala);
    }

    // Clear the shape layer
    filteredLidarLayer.destroyChildren();

    // Draw the filtered_lidar
    const FilteredLidar = new Konva.Line({
        points: filtered_lidar,
        stroke: 'orange',
        strokeWidth: 3,
    });
    filteredLidarLayer.add(FilteredLidar);

    filteredLidarLayer.draw();
}
);

// const lidarVisionTopic = new ROSLIB.Topic({
//     ros: ros,
//     name: '/detection/pointcloud',
//     messageType: 'sensor_msgs/PointCloud'
// });

// lidarVisionTopic.subscribe(function (message) {
//     const points = message.points;
//     const waypoints = [];

//     for (let i = 0; i < points.length; i++) {
//         const x = points[i].x;
//         const y = points[i].y;

//         const x_tf = x + stage.width() * 0.5 / wtf_skala;
//         const y_tf = stage.height() * 0.5 / wtf_skala - y;

//         waypoints.push(x_tf * wtf_skala, y_tf * wtf_skala);
//     }

//     // Clear the shape layer
//     lidarLayer.destroyChildren();

//     // Draw the waypoints
//     const waypointsLine = new Konva.Line({
//         points: waypoints,
//         stroke: 'blue',
//         strokeWidth: 5,
//     });
//     lidarLayer.add(waypointsLine);

//     lidarLayer.draw();
// }
// );

// ==============================================================

const waypointsTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/master/waypoints',
    messageType: 'sensor_msgs/PointCloud'
});

const waypointsKananTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/master/waypoints_kanan',
    messageType: 'sensor_msgs/PointCloud'
});

const waypointsKiriTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/master/waypoints_kiri',
    messageType: 'sensor_msgs/PointCloud'
});

const waypointsTengahTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/master/waypoints_tengah',
    messageType: 'sensor_msgs/PointCloud'
});

// Subscriptions dengan fungsi yang sama
waypointsTopic.subscribe((message) => {
    drawWaypoints(message.points, 'blue', 'utama');
});

waypointsKananTopic.subscribe((message) => {
    drawWaypoints(message.points, 'purple', 'kanan');
});

waypointsKiriTopic.subscribe((message) => {
    drawWaypoints(message.points, 'green', 'kiri');
});

waypointsTengahTopic.subscribe((message) => {
    drawWaypoints(message.points, 'yellow', 'tengah');
});

// Menyimpan referensi Line untuk setiap topik
const lines = {
    utama: null,
    kanan: null,
    kiri: null,
    tengah: null,
};

// Fungsi menggambar jalur waypoint
function drawWaypoints(points, color, key) {
    const transformedPoints = [];

    for (let i = 0; i < points.length; i++) {
        const x = points[i].x;
        const y = points[i].y;

        const x_tf = x + stage.width() * 0.5 / wtf_skala;
        const y_tf = stage.height() * 0.5 / wtf_skala - y;

        transformedPoints.push(x_tf * wtf_skala, y_tf * wtf_skala);
    }

    // Hapus Line lama jika ada
    if (lines[key]) {
        lines[key].destroy();
    }

    // Buat Line baru untuk jalur tersebut
    const newLine = new Konva.Line({
        points: transformedPoints,
        stroke: color,
        strokeWidth: 5,
        lineJoin: 'round',
    });

    waypointsLayer.add(newLine);
    lines[key] = newLine;

    waypointsLayer.draw();
}

// ==============================================================

const terminalsTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/master/terminals',
    messageType: 'ros2_interface/TerminalArray'
});

terminalsTopic.subscribe(function (message) {
    const points = message.terminals;

    // Clear the shape layer
    terminalsLayer.destroyChildren();

    // Draw terminals as outlined circles
    for (let i = 0; i < points.length; i++) {
        const x = points[i].target_pose_x;
        const y = points[i].target_pose_y;
        const radius = points[i].radius_area;

        // Transform coordinates according to your logic
        const x_tf = x + (stage.width() * 0.5) / wtf_skala;
        const y_tf = (stage.height() * 0.5) / wtf_skala - y;

        let color = 'orange'; // Default color

        if (points[i].type == 1)
            color = 'orange';
        else if (points[i].type == 2)
            color = 'yellow';

        const terminalCircle = new Konva.Circle({
            x: x_tf * wtf_skala,
            y: y_tf * wtf_skala,
            radius: radius * wtf_skala,
            stroke: color,    // Outline color
            strokeWidth: 5,      // Outline thickness
            fill: null,          // Ensure circle is not filled
        });

        // Add circle to the terminalsLayer
        terminalsLayer.add(terminalCircle);
    }

    terminalsLayer.draw();
}
);



// Create a ROSLIB Topic to subscribe to the 'ui_test' topic
const mapTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/slam/map', // Change topic name as needed
    messageType: 'nav_msgs/OccupancyGrid'
});

//===============================================================================================================================
let last_time_update_map = 0;
let mapCanvas = document.createElement("canvas");
let mapCtx = mapCanvas.getContext("2d");

// Subscribe to map messages and draw the map
mapTopic.subscribe(function (message) {
    if (new Date().getTime() - last_time_update_map < 1000) {
        return;
    }
    // console.log(message.info.origin);
    last_time_update_map = new Date().getTime();

    let map = message.data;
    let width = message.info.width;
    let height = message.info.height;

    // Resize the canvas buffer
    mapCanvas.width = width;
    mapCanvas.height = height;

    // Draw the map to the canvas buffer
    let imageData = mapCtx.createImageData(width, height);
    for (let i = 0; i < map.length; i++) {
        let occupancy = map[i];

        let r, g, b;

        if (occupancy === -1) {
            // Unknown space -> Dark Green
            r = 71;
            g = 128;
            b = 118;
        } else if (occupancy === 0) {
            // Free space -> White
            r = 255;
            g = 255;
            b = 255;
        } else {
            // Occupied space -> Color gradient (Red to Orange)
            let t = occupancy / 100; // Normalize to 0-1

            // Interpolate red and green between red (255, 0, 0) and orange (255, 165, 0)
            r = 0; // Red is fixed
            g = Math.round(165 - t * 165); // From 165 to 0 (green scale)
            b = 0; // Blue stays 0 for red-to-orange color
        }

        // Assign colors to image data
        imageData.data[i * 4 + 0] = r;  // Red
        imageData.data[i * 4 + 1] = g;  // Green
        imageData.data[i * 4 + 2] = b;  // Blue
        imageData.data[i * 4 + 3] = 255; // Fully opaque
    }
    mapCtx.putImageData(imageData, 0, 0);

    // console.log(message.info.origin.position.x, " ", message.info.origin.position.y);

    // Convert canvas to Konva image
    // x = x + stage.width() * 0.5 / wtf_skala;
    // y = stage.height() * 0.5 / wtf_skala - y;
    let imageObj = new Image();
    imageObj.onload = function () {
        let mapImage = new Konva.Image({
            image: imageObj,
            width: width * wtf_skala * message.info.resolution,
            height: height * wtf_skala * message.info.resolution,
            x: (message.info.origin.position.x + stage.width() * 0.5 / wtf_skala) * wtf_skala,
            y: (stage.height() * 0.5 / wtf_skala - message.info.origin.position.y) * wtf_skala,
        });

        // Flip Y-axis if the map's origin is bottom-left
        mapImage.scaleY(-1);
        // mapImage.offsetY(height);

        // Clear and add the new map image
        mapLayer.destroyChildren();
        mapLayer.add(mapImage);
        mapLayer.draw();
    };
    imageObj.src = mapCanvas.toDataURL();
});


// ================================================================================================================================

const topic_aruco = new ROSLIB.Topic({
    ros: ros,
    name: '/sign/marker/id',
    messageType: 'std_msgs/Int32'
});

const topic_selected_lane = new ROSLIB.Topic({
    ros: ros,
    name: '/web/selected_lane',
    messageType: 'std_msgs/Int8'
});

const topic_velocity_and_steering = new ROSLIB.Topic({
    ros: ros,
    name: '/master/ui_target_velocity_and_steering',
    messageType: 'std_msgs/Float32MultiArray'
});

let ui_target_velocity = 0.0;
let ui_target_steering = 0.0;

let key_web = 0;
let aruco_set = -1;
let selected_lane = 0;


document.addEventListener('keydown', function (event) {

    if (event.key == 'w') {
        ui_target_velocity += 0.08;
    }
    else if (event.key == 'q') {
        ui_target_velocity = 1.0;
    }
    else if (event.key == 's') {
        ui_target_velocity -= 0.08;
    }
    else if (event.key == "j") {
        ui_target_velocity = 0.8;
    }
    else if (event.key == "g") {
        ui_target_velocity = -0.6;
    }
    else if (event.key == 'm') {
        // ui_target_steering += -0.076928521 * 1 / 10;
        ui_target_steering -= 0.1;

        if (ui_target_steering < -0.57) {
            ui_target_steering = -0.57;
        }
    }
    else if (event.key == 'n') {
        // ui_target_steering = 0;
        ui_target_steering = 0.0;
    }
    else if (event.key == 'b') {
        // ui_target_steering += 0.076928521 * 1 / 10;
        ui_target_steering += 0.1;

        if (ui_target_steering > 0.57) {
            ui_target_steering = 0.57;
        }
    }
    else if (event.key == ' ') {
        ui_target_velocity = 0.0;
        ui_target_steering = 0.0;
    }
    else if (event.key == '0') {
        aruco_set = 0;
    }
    else if (event.key == '1') {
        aruco_set = 1;
    }
    else if (event.key == '2') {
        aruco_set = 2;
    }
    else if (event.key == '3') {
        aruco_set = 3;
    }
    else if (event.key == '4') {
        aruco_set = 4;
    }
    else if (event.key == '5') {
        aruco_set = 5;
    }
    else if (event.key == '-') {
        ui_target_steering = 0.38;
    }
    else if (event.key == '`') {
        aruco_set = -1;
    }
    else if (event.key == 'u') {
        selected_lane = -1;
    }
    else if (event.key == 'i') {
        selected_lane = 0;
    }
    else if (event.key == 'o') {
        selected_lane = 1;
    }

    topic_selected_lane.publish({ data: selected_lane });
    topic_aruco.publish({ data: aruco_set });
    topic_velocity_and_steering.publish({ data: [ui_target_velocity, ui_target_steering] });
});

// ================================================================================================================================

function add_terminal_here(add_terminal) {
    const modeRequest = new ROSLIB.ServiceRequest({
        data: add_terminal
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/master/set_terminal',
        serviceType: 'std_srvs/srv/SetBool'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Set Terminal:', response);
    });
}

function add_terminal_here_stop(add_terminal) {
    const modeRequest = new ROSLIB.ServiceRequest({
        data: add_terminal
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/master/set_terminal_sign',
        serviceType: 'std_srvs/srv/SetBool'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Set Terminal:', response);
    });
}




function remove_terminal(remove_all) {
    const modeRequest = new ROSLIB.ServiceRequest({
        data: remove_all
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/master/rm_terminal',
        serviceType: 'std_srvs/srv/SetBool'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Set Terminal:', response);
    });
}

function rtabmap_set_mode_mapping() {
    const modeRequest = new ROSLIB.ServiceRequest({
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/slam/rtabmap/set_mode_mapping',
        serviceType: 'std_srvs/srv/Empty'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Set mode mapping:', response);
    });
}

function rtabmap_set_mode_localization() {
    const modeRequest = new ROSLIB.ServiceRequest({
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/slam/rtabmap/set_mode_localization',
        serviceType: 'std_srvs/srv/Empty'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Set mode localization:', response);
    });
}



function rtabmap_trigger_new_map() {
    const modeRequest = new ROSLIB.ServiceRequest({
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/slam/rtabmap/trigger_new_map',
        serviceType: 'std_srvs/srv/Empty'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('New Map:', response);
    });
}
function rtabmap_reset() {
    offset_odometry(0, 0, 0);

    // Delay 
    setTimeout(() => {
    }, 2000); // 2000ms = 2 seconds    

    const modeRequest = new ROSLIB.ServiceRequest({
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/slam/rtabmap/reset',
        serviceType: 'std_srvs/srv/Empty'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Reset:', response);
    });
}

function rtabmap_reset_map2odom(x, y, theta) {
    // Initialize ROSLIB topic publisher for initialpose
    const initialPoseTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/rtabmap/initialpose',  // Typically this topic is used for initial pose
        messageType: 'geometry_msgs/PoseWithCovarianceStamped'
    });

    // Construct the message
    const initialPoseMsg = new ROSLIB.Message({
        header: {
            seq: 0,
            stamp: { secs: 0, nsecs: 0 },
            frame_id: "map" // or the appropriate frame your RTAB-Map uses
        },
        pose: {
            pose: {
                position: { x: x, y: y, z: 0.0 }, // position in meters
                orientation: {
                    x: 0, y: 0, z: Math.sin(theta / 2), w: Math.cos(theta / 2) // quaternion
                }
            },
            covariance: [  // Standard covariance (can be identity or small values)
                0.25, 0, 0, 0, 0, 0,
                0, 0.25, 0, 0, 0, 0,
                0, 0, 0.25, 0, 0, 0,
                0, 0, 0, 0.068, 0, 0,
                0, 0, 0, 0, 0.068, 0,
                0, 0, 0, 0, 0, 0.068
            ]
        }
    });

    // Publish the message
    initialPoseTopic.publish(initialPoseMsg);
    console.log('Published initial pose (0,0,0)');
}

function route_record(record) {
    const modeRequest = new ROSLIB.ServiceRequest({
        data: record
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/master/set_record_route_mode',
        serviceType: 'std_srvs/srv/SetBool'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Record:', response);
    });
}

function route_record_kanan(record) {
    const modeRequest = new ROSLIB.ServiceRequest({
        data: record
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/master/set_record_route_mode_kanan',
        serviceType: 'std_srvs/srv/SetBool'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Record:', response);
    });
}

function route_record_kiri(record) {
    const modeRequest = new ROSLIB.ServiceRequest({
        data: record
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/master/set_record_route_mode_kiri',
        serviceType: 'std_srvs/srv/SetBool'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Record:', response);
    });
}



function add_route_record(add_record) {
    const modeRequest = new ROSLIB.ServiceRequest({
        data: add_record
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/master/set_add_record_route_mode',
        serviceType: 'std_srvs/srv/SetBool'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Record:', response);
    });
}

function offset_odometry(x, y, theta) {
    var pub_pose_offset = new ROSLIB.Topic({
        ros: ros,
        name: '/master/pose_offset',
        messageType: 'nav_msgs/msg/Odometry'
    });

    // Construct the Odometry message
    var msg_pose_offset = new ROSLIB.Message({
        header: {
            stamp: { sec: 0, nanosec: 0 },  // Set timestamps dynamically if needed
            frame_id: "pose_offset"
        },
        child_frame_id: "base_link",
        pose: {
            pose: {
                position: { x: x, y: y, z: 0.0 },
                orientation: { x: 0, y: 0, z: theta, w: 1 }
            }
        },
        twist: {
            twist: {
                linear: { x: 0.0, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0.0 }
            }
        }
    });
    pub_pose_offset.publish(msg_pose_offset);
}

const topic_fsm = new ROSLIB.Topic({
    ros: ros,
    name: '/master/set_master_fsm',
    messageType: 'std_msgs/Int16'
});


function set_master_fsm(target_fsm) {

    topic_fsm.publish({ data: target_fsm });
}






const global_fsm = document.getElementById('global-fsm');
var sub_master_global_fsm = new ROSLIB.Topic({
    ros: ros,
    name: "/master/global_fsm",
    messageType: "std_msgs/Int16",
});

sub_master_global_fsm.subscribe(function (message) {
    if (message.data == 0) {
        global_fsm.innerHTML = "INIT"
    }
    else if (message.data == 1) {
        global_fsm.innerHTML = "Pre-Operation"
    }
    else if (message.data == 2) {
        global_fsm.innerHTML = "Safe-Operation"
    }
    else if (message.data == 3) {
        global_fsm.innerHTML = "Operation Mode 3"
    }
    else if (message.data == 4) {
        global_fsm.innerHTML = "Operation Mode 4"
    }
    else if (message.data == 5) {
        global_fsm.innerHTML = "Operation Mode 5"
    }
    else if (message.data == 6) {
        global_fsm.innerHTML = "Operation Mode 2"
    }
    else if (message.data == 7) {
        global_fsm.innerHTML = "Routing Mode"
    }
    else if (message.data == 8) {
        global_fsm.innerHTML = "Mapping Mode"
    } else if (message.data == 200) {
        global_fsm.innerHTML = "Race Mode"
    }
    else if (message.data == 100) {
        global_fsm.innerHTML = "Urban Mode"
    }
});




// ================================================================

// ================================================================


// setInterval(() => {
// }, 50);
