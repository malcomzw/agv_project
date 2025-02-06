pipeline {
    agent any

    environment {
        DOCKER_IMAGE = 'ros-jenkins'
        DOCKER_TAG = "${BUILD_ID}"
        ROS_WORKSPACE = "${WORKSPACE}/ros_ws"
    }

    stages {
        stage('Build Docker') {
            steps {
                sh "docker build -t ${DOCKER_IMAGE}:${DOCKER_TAG} -f docker/Dockerfile ."
            }
        }

        stage('Initialize Workspace') {
            steps {
                sh """
                    docker run --rm \
                        -v ${WORKSPACE}:/workspace \
                        -w /workspace/ros_ws \
                        ${DOCKER_IMAGE}:${DOCKER_TAG} \
                        bash -c '
                            source /opt/ros/noetic/setup.bash && \
                            rm -rf build devel install && \
                            catkin init && \
                            catkin config --install'
                """
            }
        }

        stage('Build ROS') {
            steps {
                sh """
                    docker run --rm \
                        -v ${WORKSPACE}:/workspace \
                        -w /workspace/ros_ws \
                        ${DOCKER_IMAGE}:${DOCKER_TAG} \
                        bash -c '
                            source /opt/ros/noetic/setup.bash && \
                            catkin build --no-status'
                """
            }
        }

        stage('Test') {
            steps {
                sh """
                    docker run --rm \
                        -v ${WORKSPACE}:/workspace \
                        -w /workspace/ros_ws \
                        ${DOCKER_IMAGE}:${DOCKER_TAG} \
                        bash -c '
                            source /opt/ros/noetic/setup.bash && \
                            source devel/setup.bash && \
                            catkin test demo_pkg --no-deps'
                """
            }
        }
    }

    post {
        always {
            sh "docker rmi ${DOCKER_IMAGE}:${DOCKER_TAG} || true"
            cleanWs()
        }
    }
}
