// Khởi tạo ứng dụng Vue
var app = new Vue({
    el: '#app',
    data: {
        index: '2',
        connected: false,
        ros: null,
        ws_address: 'ws://192.168.2.83:9090',
        logs: [],
        loading: false,
        zoomEnabled: false, // Thêm thuộc tính để kiểm tra xem chế độ zoom bằng chuột có được kích hoạt không
        dragging: false, // Thêm thuộc tính để kiểm tra xem có đang kéo thả chuột không
        dragStartX: 0,
        dragStartY: 0,
        dragShiftX: 0,
        dragShiftY: 0,
    },
    mounted() {
        this.init();
        window.addEventListener('keydown', this.handleKeyDown);
        window.addEventListener('keyup', this.handleKeyUp);
        window.addEventListener('wheel', this.handleMouseWheel);
        window.addEventListener('mousedown', this.handleMouseDown);
        window.addEventListener('mouseup', this.handleMouseUp);
        window.addEventListener('mousemove', this.handleMouseMove);
    },
    methods: {
        handleKeyDown(event) {
            // Xử lý sự kiện nhấn phím
            switch (event.key) {
                case 'a':
                    this.turnLeft();
                    break;
                case 'w':
                    this.forward();
                    break;
                case 's':
                    this.stop();
                    break;
                case 'd':
                    this.turnRight();
                    break;
                case 'x':
                    this.backWard();
                    break;
            }
        },
        handleKeyUp(event) {
            // Xử lý sự kiện thả phím
            switch (event.key) {
                case 'a':
                case 'w':
                case 's':
                case 'd':
                case 'x':
                    this.stop();
                    break;
            }
        },
        init() {
            // Kết nối đến ROS.
            this.ros = new ROSLIB.Ros({
                url: this.ws_address
            });

            // Tạo người xem chính.
            this.viewer = new ROS2D.Viewer({
                divID: 'map',
                width: 640,
                height: 480
            });

            // Thiết lập client bản đồ.
            this.gridClient = new ROS2D.OccupancyGridClient({
                ros: this.ros,
                rootObject: this.viewer.scene,
                continuous: true
            });

            // Điều chỉnh kích thước canvas để vừa với bản đồ
            this.gridClient.on('change', () => {
                this.viewer.scaleToDimensions(this.gridClient.currentGrid.width, this.gridClient.currentGrid.height);
                this.viewer.shift(this.gridClient.currentGrid.pose.position.x, this.gridClient.currentGrid.pose.position.y);
            });

            // Set topic cho pose
            this.setTopic();

            // Đăng ký xử lý cho topic poseTopic
            this.poseTopic.subscribe(function (message) {
                var pose = message.pose.pose;

                // Cập nhật vị trí và góc quay cho robot marker
                robotMarker.x = pose.position.x;
                robotMarker.y = pose.position.y;
                robotMarker.rotation = Math.atan2(pose.orientation.z, pose.orientation.w) * 2;

                // Hiển thị robot marker trên bản đồ
                this.rootObject.addChild(robotMarker);
            });
        },
        connect: function () {
            // Kết nối và ngắt kết nối với ROS
            this.logs.unshift('connect to rosbridge server!!');
            this.ros = new ROSLIB.Ros({
                url: this.ws_address
            });

            this.ros.on('connection', () => {
                this.logs.unshift('Connected');
                this.connected = true;
            });

            this.ros.on('error', (error) => {
                this.logs.unshift('Error connecting to WebSocket server:');
            });

            this.ros.on('close', () => {
                this.connected = false;
                this.logs.unshift('WebSocket server connection closed.');
            });
        },
        disconnect: function () {
            // Ngắt kết nối với ROS
            this.ros.close();
        },
        setTopic: function () {
            // Thiết lập topic
            this.topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            });

            this.publisher = new ROSLIB.Topic({
                ros: this.ros,
                name: '/subscriber',
                messageType: 'std_msgs/String'
            });
            this.save_location = new ROSLIB.Topic({
                ros: this.ros,
                name: '/save_request',
                messageType: 'std_msgs/String'
            });

            this.poseTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/odom',
                messageType: 'nav_msgs/Odometry'
            });
        },
        run_saveLocation: function () {
            // Tạo message với giá trị index hiện tại
            this.message = new ROSLIB.Message({
                data: this.index
            });

            // Publish message
            this.setTopic();
            this.publisher.publish(this.message);
        },
        Navigation: function () {
            // Tạo message để lưu bản đồ
            this.message = new ROSLIB.Message({
                data: '4'
            });

            // Publish message để lưu bản đồ
            this.setTopic();
            this.publisher.publish(this.message);
        },
        saveLocation: function () {
            // Tạo message để lưu bản đồ
            this.message = new ROSLIB.Message({
                data: 'save'
            });

            // Publish message để lưu bản đồ
            this.setTopic();
            this.save_location.publish(this.message);

        },

        forward: function () {
            // Di chuyển robot về phía trước
            this.message = new ROSLIB.Message({
                linear: { x: 0.22, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0 }
            });
            this.setTopic();
            this.topic.publish(this.message);
        },
        turnLeft: function () {
            // Quay robot sang trái
            this.message = new ROSLIB.Message({
                linear: { x: 0.22, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0.3 }
            });
            this.setTopic();
            this.topic.publish(this.message);
        },
        turnRight: function () {
            // Quay robot sang phải
            this.message = new ROSLIB.Message({
                linear: { x: 0.22, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: -0.3 }
            });
            this.setTopic();
            this.topic.publish(this.message);
        },
        backWard: function () {
            // Di chuyển robot về phía sau
            this.message = new ROSLIB.Message({
                linear: { x: -0.3, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0 }
            });
            this.setTopic();
            this.topic.publish(this.message);
        },
        stop: function () {
            // Dừng robot
            this.message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0 }
            });
            this.setTopic();
            this.topic.publish(this.message);
        },
        zoomIn: function () {
            // Thu phóng bản đồ
            this.viewer.scale *= 1.2;
            var zoom = new ROS2D.ZoomView({
                ros: this.ros,
                rootObject: this.viewer.scene
            });
            zoom.startZoom(0, 0);
            zoom.zoom(1.2);
        },
        zoomOut: function () {
            // Thu nhỏ bản đồ
            this.viewer.scale *= 0.8;
            // Thêm logic cho phương thức zoomOut
            var zoom = new ROS2D.ZoomView({
                ros: this.ros,
                rootObject: this.viewer.scene
            });
            zoom.startZoom(0, 0);
            zoom.zoom(0.8);
        },
        handleMouseWheel(event) {
            // Kích hoạt chế độ zoom bằng chuột khi di chuột vào bản đồ
            this.zoomEnabled = true;
            // if (event.deltaY > 0) {
            //     this.zoomOut();
            // } else {
            //     this.zoomIn();
            // }
        },
        handleMouseDown(event) {
            // Bắt đầu kéo thả chuột
            if (this.zoomEnabled) {
                this.dragging = true;
                this.dragStartX = event.clientX;
                this.dragStartY = event.clientY;
                this.dragShiftX = this.viewer.scene.x;
                this.dragShiftY = this.viewer.scene.y;
            }
        },
        handleMouseUp(event) {
            // Kết thúc kéo thả chuột
            if (this.zoomEnabled) {
                this.dragging = false;
            }
        },
        handleMouseMove(event) {
            // Di chuyển bản đồ khi kéo thả chuột
            if (this.zoomEnabled && this.dragging) {
                const deltaX = event.clientX - this.dragStartX;
                const deltaY = event.clientY - this.dragStartY;
                this.viewer.scene.x = this.dragShiftX + deltaX;
                this.viewer.scene.y = this.dragShiftY + deltaY;
            }
        },
    },
});
