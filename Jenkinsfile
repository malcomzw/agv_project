pipeline {
    agent any

    environment {
        DOCKER_IMAGE = 'ros-jenkins'
        DOCKER_TAG = "${BUILD_ID}"
        ROS_WORKSPACE = "${WORKSPACE}/ros_ws"
    }

    stages {
        stage('Prepare Workspace') {
            steps {
                checkout scm
            }
        }

        stage('Build Docker') {
            steps {
                sh "docker build -t ${DOCKER_IMAGE}:${DOCKER_TAG} -f docker/Dockerfile ."
            }
        }

        stage('Build ROS') {
            steps {
                sh """
                    docker run --rm \
                        -v ${WORKSPACE}:/workspace \
                        -w /workspace/ros_ws \
                        ${DOCKER_IMAGE}:${DOCKER_TAG} \
                        /bin/bash -c '
                            source /opt/ros/noetic/setup.bash && \
                            rm -rf build devel install logs && \
                            catkin init && \
                            catkin config --install && \
                            catkin build
                        '
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
                        /bin/bash -c '
                            source /opt/ros/noetic/setup.bash && \
                            source install/setup.bash && \
                            catkin test demo_pkg --no-deps
                        '
                """
            }
            post {
                always {
                    junit allowEmptyResults: true, testResults: '${ROS_WORKSPACE}/build/test_results/**/*.xml'
                }
            }
        }
    }

    post {
        always {
            cleanWs()
            sh "docker rmi ${DOCKER_IMAGE}:${DOCKER_TAG} || true"
        }
    }
}
