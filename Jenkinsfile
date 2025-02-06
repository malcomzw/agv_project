pipeline {
    agent any

    environment {
        DOCKER_IMAGE = 'ros-jenkins'
        DOCKER_TAG = "${BUILD_ID}"
        ROS_WORKSPACE = "${WORKSPACE}/ros_ws"
        BASH_SOURCE = '/bin/bash'
    }

    stages {
        stage('Prepare Workspace') {
            steps {
                checkout scm
            }
        }

        stage('Build') {
            steps {
                script {
                    // Build Docker image
                    docker.build("${DOCKER_IMAGE}:${DOCKER_TAG}", '-f docker/Dockerfile .')
                }
            }
        }

        stage('Test') {
            steps {
                script {
                    docker.image("${DOCKER_IMAGE}:${DOCKER_TAG}").inside('--entrypoint="" --network=host') {
                        sh '''
                            #!/bin/bash
                            set -e
                            . /opt/ros/noetic/setup.bash
                            cd ${ROS_WORKSPACE}
                            catkin init
                            catkin clean -y
                            catkin build
                            . devel/setup.bash
                            catkin test demo_pkg --no-deps
                        '''
                    }
                }
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
