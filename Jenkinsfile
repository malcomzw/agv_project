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
                        find build -type d -name test_results -exec cp -r {} /workspace/ros_ws/test_results/ \\; || true &&
                        find build -name '*.xml' -exec cp {} /workspace/ros_ws/test_results/ \\; || true"
                    '''
                }
            }
            post {
                always {
                    script {
                        sh '''
                        echo "=== DEBUG: Test Results Directories ==="
                        find /var/lib/jenkins/workspace/ros_pipeline1/ros_ws -type d -name test_results
                        echo "=== DEBUG: XML Files Found ==="
                        find /var/lib/jenkins/workspace/ros_pipeline1/ros_ws -name "*.xml"
                        '''
                    }
                    junit allowEmptyResults: true, testResults: 'ros_ws/test_results/**/*.xml'
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

        stage('Gazebo Simulation Deployment') {
            steps {
                script {
                    catchError(buildResult: 'UNSTABLE', stageResult: 'FAILURE') {
                        sh '''
                            docker run --rm -v /var/lib/jenkins/workspace/ros_pipeline1:/workspace -w /workspace/ros_ws \
                            --env DISPLAY=:99 \
                            --env LIBGL_ALWAYS_SOFTWARE=1 \
                            --env ROS_MASTER_URI=http://localhost:11311 \
                            --env ROS_HOSTNAME=localhost \
                            ros-jenkins:91 timeout 300 /bin/bash -c "
                                set -e

                                echo '=== Initializing rosdep ==='
                                if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
                                    rosdep init
                                fi
                                rosdep update

                                echo '=== Installing Required Packages ==='
                                apt-get update
                                apt-get install -y xvfb python3-pygame mesa-utils gazebo11

                                echo '=== Resolving ROS Dependencies ==='
                                rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:focal

                                echo '=== Checking ROS Environment ==='
                                source /opt/ros/noetic/setup.bash
                                source devel/setup.bash
                                env | grep ROS

                                echo '=== Starting Virtual Display ==='
                                Xvfb :99 -screen 0 1024x768x16 &
                                export DISPLAY=:99

                                echo '=== Setting Gazebo Environment Variables ==='
                                export GAZEBO_MODEL_PATH=/workspace/ros_ws/src/agv_sim/models:${GAZEBO_MODEL_PATH}
                                export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH}
                                export GAZEBO_PLUGIN_PATH=/workspace/ros_ws/src/agv_sim/plugins:${GAZEBO_PLUGIN_PATH}
                                source /usr/share/gazebo/setup.sh

                                echo '=== Checking Gazebo Plugins ==='
                                ls -la /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/

                                echo '=== Debugging Gazebo Paths ==='
                                echo 'GAZEBO_MODEL_PATH=' $GAZEBO_MODEL_PATH
                                echo 'GAZEBO_RESOURCE_PATH=' $GAZEBO_RESOURCE_PATH
                                echo 'GAZEBO_PLUGIN_PATH=' $GAZEBO_PLUGIN_PATH
                                ls -la /usr/share/gazebo-11/worlds/

                                echo '=== Verifying Gazebo Server Startup ==='
                                gzserver --verbose /usr/share/gazebo-11/worlds/empty.world 2>&1 | tee /workspace/gzserver.log
                                GZSERVER_PID=$!

                                sleep 10

                                if ! kill -0 $GZSERVER_PID 2>/dev/null; then
                                    echo 'ERROR: Gazebo server failed to start!'
                                    echo '=== Gazebo Server Log Contents ==='
                                    cat /workspace/gzserver.log
                                    exit 1
                                fi

                                echo '=== Starting Gazebo Simulation ==='
                                roslaunch agv_sim simulation.launch use_rviz:=false gui:=false record:=true --screen --wait
                            "
                        '''
                    }
                }
                script {
                    if (fileExists('ros_ws/simulation_results')) {
                        archiveArtifacts artifacts: 'ros_ws/simulation_results/**/*', fingerprint: true
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
