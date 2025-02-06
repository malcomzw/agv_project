pipeline {
    agent any

    environment {
        DOCKER_IMAGE = 'ros-jenkins'
        DOCKER_TAG = "${BUILD_ID}"
        WORKSPACE_DIR = "${WORKSPACE}"
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
                    docker.build("${DOCKER_IMAGE}:${DOCKER_TAG}", '-f ${WORKSPACE_DIR}/docker/Dockerfile ${WORKSPACE_DIR}')
                }
            }
        }

        stage('Test') {
            agent {
                docker {
                    image "${DOCKER_IMAGE}:${DOCKER_TAG}"
                    args '--network=host'
                    reuseNode true
                }
            }
            steps {
                script {
                    sh '''
                        #!/bin/bash
                        source /opt/ros/noetic/setup.bash
                        cd ${WORKSPACE_DIR}/ros_ws
                        catkin build
                        source devel/setup.bash
                        catkin test demo_pkg --no-deps
                    '''
                }
            }
            post {
                always {
                    junit allowEmptyResults: true, testResults: '${WORKSPACE_DIR}/ros_ws/build/test_results/**/*.xml'
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
