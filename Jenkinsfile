pipeline {
    agent any

    environment {
        DOCKER_IMAGE = 'ros-jenkins'
        DOCKER_TAG = "${BUILD_ID}"
    }

    stages {
        stage('Build and Test') {
            steps {
                checkout scm
                
                // Build Docker image
                sh "docker build -t ${DOCKER_IMAGE}:${DOCKER_TAG} -f docker/Dockerfile ."
                
                // Build and test demo_pkg
                sh """
                    docker run --rm \
                        -v ${WORKSPACE}:/workspace \
                        -w /workspace/ros_ws \
                        ${DOCKER_IMAGE}:${DOCKER_TAG} \
                        /bin/bash -c '
                            source /opt/ros/noetic/setup.bash && \
                            rm -rf build devel && \
                            mkdir -p build devel && \
                            catkin_make --pkg demo_pkg && \
                            source devel/setup.bash && \
                            catkin_make run_tests_demo_pkg
                        '
                """
            }
            post {
                always {
                    junit allowEmptyResults: true, testResults: 'ros_ws/build/test_results/demo_pkg/*.xml'
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
