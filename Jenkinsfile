pipeline {
    agent any
    
    environment {
        DOCKER_IMAGE_TAG = "ros-jenkins:93"
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
                    sh 'git rev-parse --short HEAD'
                    sh 'echo Build from commit: $(git rev-parse --short HEAD)'
                    sh 'echo On branch: $(git rev-parse --abbrev-ref HEAD)'
                    sh 'git show -s --format=%B HEAD'
                }
            }
        }
        
        stage('Workspace Info') {
            steps {
                sh 'echo Workspace contents:'
                sh 'ls -la'
                sh 'echo'
                sh 'echo Disk space:'
                sh 'df -h .'
            }
        }
        
        stage('Build Docker Image') {
            steps {
                sh "docker build -t ${env.DOCKER_IMAGE_TAG} -f docker/Dockerfile ."
            }
        }
        
        stage('Build ROS Package') {
            steps {
                sh """
                docker run --rm \
                    -v /var/lib/jenkins/workspace/ros_pipeline1:/workspace \
                    -w /workspace/ros_ws \
                    ${env.DOCKER_IMAGE_TAG} \
                    /bin/bash -c '
                        source /opt/ros/noetic/setup.bash && \
                        rm -rf .catkin_tools build devel logs && \
                        catkin init && \
                        catkin clean -y && \
                        catkin build \
                            --summarize \
                            --no-status \
                            --force-color \
                            --cmake-args -DCMAKE_BUILD_TYPE=Release
                    '
                """
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
                sh """
                docker run --rm \
                    -v /var/lib/jenkins/workspace/ros_pipeline1:/workspace \
                    -w /workspace/ros_ws \
                    ${env.DOCKER_IMAGE_TAG} \
                    /bin/bash -c '
                        source /opt/ros/noetic/setup.bash && \
                        source devel/setup.bash && \
                        catkin_test_results build/test_results
                    '
                """
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
                        sh """
                        docker run --rm \
                            -v /var/lib/jenkins/workspace/ros_pipeline1:/workspace \
                            -w /workspace/ros_ws \
                            --env DISPLAY=:99 \
                            --env ROS_MASTER_URI=http://localhost:11311 \
                            --env ROS_HOSTNAME=localhost \
                            ${env.DOCKER_IMAGE_TAG} \
                            timeout 300 /bin/bash -c "
                                set -e
                                
                                source /opt/ros/noetic/setup.bash
                                source devel/setup.bash
                                
                                # Setup virtual framebuffer
                                Xvfb :99 -screen 0 1024x768x16 & 
                                export DISPLAY=:99
                                
                                # Set Gazebo paths
                                export GAZEBO_RESOURCE_PATH=/workspace/ros_ws/src/agv_sim/worlds
                                export GAZEBO_MODEL_PATH=/workspace/ros_ws/src/agv_sim/models
                                
                                echo '=== Preparing Simulation Environment ==='
                                roscore & 
                                ROSCORE_PID=$!
                                sleep 5
                                
                                echo '=== Launching AGV Simulation ==='
                                roslaunch agv_sim simulation.launch gui:=false record:=true &
                                SIMULATION_PID=$!
                                
                                echo '=== Waiting for Simulation Startup ==='
                                sleep 30
                                
                                echo '=== Running Simulation Tests ==='
                                rostest agv_sim simulation_test.test || true
                                
                                echo '=== Generating Simulation Report ==='
                                mkdir -p /workspace/ros_ws/simulation_results
                                
                                # Capture rosbag data
                                rosbag record -O /workspace/ros_ws/simulation_results/simulation_data.bag \
                                    /tf /tf_static /odom /cmd_vel /scan /rosout -D 60 &
                                ROSBAG_PID=$!
                                
                                sleep 60
                                
                                # Kill processes
                                kill $ROSBAG_PID $SIMULATION_PID $ROSCORE_PID || true
                                
                                # Generate simulation performance metrics
                                echo 'Simulation Performance Metrics:' > /workspace/ros_ws/simulation_results/performance_report.txt
                                echo '--------------------------------' >> /workspace/ros_ws/simulation_results/performance_report.txt
                                rostopic hz /odom >> /workspace/ros_ws/simulation_results/performance_report.txt 2>&1
                                
                                # Generate simulation log summary
                                grep -R 'error|warning' /root/.ros/log/ > /workspace/ros_ws/simulation_results/simulation_log_summary.txt || true
                            "
                        """
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
