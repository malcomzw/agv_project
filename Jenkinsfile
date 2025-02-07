pipeline {
    agent any

    options {
        timestamps()
        timeout(time: 5, unit: 'MINUTES')
        skipDefaultCheckout()
        disableConcurrentBuilds()
    }

    environment {
        GIT_COMMIT_SHORT = ''
    }

    stages {
        stage('Cleanup') {
            steps {
                cleanWs()
            }
        }

        stage('Checkout') {
            steps {
                checkout scm
                script {
                    env.GIT_COMMIT_SHORT = sh(script: 'git rev-parse --short HEAD', returnStdout: true).trim()
                    sh '''
                        echo "Build from commit: ${GIT_COMMIT_SHORT}"
                        echo "On branch: $(git rev-parse --abbrev-ref HEAD)"
                        git show -s --format=%B HEAD
                    '''
                }
            }
        }

        stage('Workspace Info') {
            steps {
                sh '''
                    echo "Workspace contents:"
                    ls -la
                    echo "\nDisk space:"
                    df -h .
                '''
            }
        }

        stage('Build Docker Image') {
            steps {
                sh 'docker build -t ros-jenkins:${BUILD_ID} -f docker/Dockerfile .'
            }
        }

        stage('Build ROS Package') {
            steps {
                sh '''
                    docker run --rm \
                        -v ${WORKSPACE}:/workspace \
                        -w /workspace/ros_ws \
                        ros-jenkins:${BUILD_ID} \
                        /bin/bash -c '
                            source /opt/ros/noetic/setup.bash && \
                            rm -rf .catkin_tools build devel logs && \
                            catkin init && \
                            catkin clean -y && \
                            catkin build --summarize \
                                --no-status \
                                --force-color \
                                --cmake-args -DCMAKE_BUILD_TYPE=Release
                        '
                '''
            }
            post {
                success {
                    echo 'ROS package built successfully!'
                }
                failure {
                    echo 'Failed to build ROS package'
                }
            }
        }

        stage('Run Tests') {
            steps {
                script {
                    sh '''
                    docker run --rm -v /var/lib/jenkins/workspace/ros_pipeline1:/workspace -w /workspace/ros_ws ros-jenkins:80 /bin/bash -c "
                        source /opt/ros/noetic/setup.bash &&
                        source devel/setup.bash &&
                        mkdir -p build/test_results test_results &&
                        catkin run_tests --no-deps &&
                        catkin_test_results build/test_results --verbose > test_results/summary.txt &&
                        find build/test_results -name "*.xml" -exec cp {} test_results/ \\; || true"
                    '''
                }
            }
            post {
                always {
                    script {
                        // Additional debugging
                        sh '''
                            echo "=== DEBUG: Build Test Results Directories ==="
                            find ros_ws/build -type d -name "test_results"
                            echo "=== DEBUG: XML Files Found ==="
                            find ros_ws -name "*.xml"
                            echo "=== DEBUG: Test Results Directory Contents ==="
                            ls -la ros_ws/test_results/ || true
                        '''
                    }
                    archiveArtifacts artifacts: 'ros_ws/test_results/**/*', allowEmptyArchive: true
                    junit testResults: 'ros_ws/test_results/**/*.xml', allowEmptyResults: true
                }
                success {
                    echo 'All tests passed!'
                }
                failure {
                    echo 'Some tests failed!'
                    script {
                        // Detailed failure debugging
                        sh '''
                            echo "=== FAILURE DEBUGGING ==="
                            echo "Contents of test_results directory:"
                            ls -la ros_ws/test_results/ || true
                            echo "\n=== Detailed XML Search ==="
                            find ros_ws -name "*.xml" -ls || true
                        '''
                    }
                }
            }
        }
    }

    post {
        success {
            echo "Build succeeded! Commit: ${env.GIT_COMMIT_SHORT}"
        }
        failure {
            echo "Build failed! Commit: ${env.GIT_COMMIT_SHORT}"
        }
        always {
            echo 'Pipeline completed.'
            junit 'ros_ws/test_results/**/*.xml'
            cleanWs()
        }
    }
}
