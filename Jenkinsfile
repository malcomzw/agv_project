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
            environment {
                SIMULATION_TIMEOUT = 300  // 5 minutes max simulation time
            }
            steps {
                script {
                    catchError(buildResult: 'UNSTABLE', stageResult: 'FAILURE') {
                        sh '''
                        docker run --rm \
                            -v /var/lib/jenkins/workspace/ros_pipeline1:/workspace \
                            -w /workspace/ros_ws \
                            --env DISPLAY=:99 \
                            --env ROS_MASTER_URI=http://localhost:11311 \
                            --env ROS_HOSTNAME=localhost \
                            ros-jenkins:80 \
                            timeout ${SIMULATION_TIMEOUT} /bin/bash -c "
                                set -e  # Exit immediately if a command exits with a non-zero status
                                
                                source /opt/ros/noetic/setup.bash &&
                                source devel/setup.bash &&
                                
                                # Setup virtual framebuffer
                                Xvfb :99 & 
                                export DISPLAY=:99
                                
                                # Set Gazebo paths
                                export GAZEBO_RESOURCE_PATH=/workspace/ros_ws/src/agv_sim/worlds:$GAZEBO_RESOURCE_PATH &&
                                export GAZEBO_MODEL_PATH=/workspace/ros_ws/src/agv_sim/models:$GAZEBO_MODEL_PATH &&
                                
                                echo '=== Preparing Simulation Environment ===' &&
                                roscore & 
                                ROSCORE_PID=$! &&
                                sleep 5 &&
                                
                                echo '=== Launching AGV Simulation ===' &&
                                roslaunch agv_sim simulation.launch gui:=false record:=true &
                                SIMULATION_PID=$! &&
                                
                                echo '=== Waiting for Simulation Startup ===' &&
                                sleep 30 &&  # Allow simulation to stabilize
                                
                                echo '=== Running Simulation Tests ===' &&
                                rostest agv_sim simulation_integration_test.test || true &&
                                
                                echo '=== Generating Simulation Report ===' &&
                                mkdir -p /workspace/ros_ws/simulation_results &&
                                
                                # Capture rosbag data
                                rosbag record -O /workspace/ros_ws/simulation_results/simulation_data.bag \
                                    /tf /tf_static /odom /cmd_vel /scan /rosout -d 60 &
                                ROSBAG_PID=$! &&
                                
                                sleep 60 &&  # Record for 60 seconds
                                
                                # Kill processes
                                kill $ROSBAG_PID $SIMULATION_PID $ROSCORE_PID || true &&
                                
                                # Generate simulation performance metrics
                                echo 'Simulation Performance Metrics:' > /workspace/ros_ws/simulation_results/performance_report.txt &&
                                echo '--------------------------------' >> /workspace/ros_ws/simulation_results/performance_report.txt &&
                                rostopic hz /odom >> /workspace/ros_ws/simulation_results/performance_report.txt 2>&1 &&
                                
                                # Generate simulation log summary
                                grep -R "error\\|warning" /root/.ros/log/ > /workspace/ros_ws/simulation_results/simulation_log_summary.txt || true
                            "'
                        
                        # Copy simulation results back to Jenkins workspace
                        mkdir -p ros_ws/simulation_results
                        docker cp $(docker ps -lq):/workspace/ros_ws/simulation_results/. ros_ws/simulation_results/ || true
                        '''
                    }
                }
            }
            post {
                always {
                    script {
                        // Check if simulation results exist
                        def simulationResultsExist = fileExists 'ros_ws/simulation_results'
                        
                        if (simulationResultsExist) {
                            // Archive artifacts
                            archiveArtifacts artifacts: 'ros_ws/simulation_results/**/*', allowEmptyArchive: true
                            
                            // Generate summary report
                            sh '''
                            echo "=== Gazebo Simulation Report ===" > ros_ws/simulation_results/simulation_summary.txt
                            echo "Date: $(date)" >> ros_ws/simulation_results/simulation_summary.txt
                            echo "Build Number: ${BUILD_NUMBER}" >> ros_ws/simulation_results/simulation_summary.txt
                            echo "" >> ros_ws/simulation_results/simulation_summary.txt
                            
                            echo "Performance Metrics:" >> ros_ws/simulation_results/simulation_summary.txt
                            cat ros_ws/simulation_results/performance_report.txt >> ros_ws/simulation_results/simulation_summary.txt
                            
                            echo "" >> ros_ws/simulation_results/simulation_summary.txt
                            echo "Simulation Logs Summary:" >> ros_ws/simulation_results/simulation_summary.txt
                            cat ros_ws/simulation_results/simulation_log_summary.txt >> ros_ws/simulation_results/simulation_summary.txt
                            '''
                            
                            // Publish HTML report
                            publishHTML(
                                target: [
                                    allowMissing: true,
                                    alwaysLinkToLastBuild: true,
                                    keepAll: true,
                                    reportDir: 'ros_ws/simulation_results',
                                    reportFiles: 'simulation_summary.txt',
                                    reportName: 'Gazebo Simulation Report'
                                ]
                            )
                        } else {
                            echo "No simulation results found. Simulation may have failed."
                        }
                    }
                }
                
                failure {
                    echo "Gazebo Simulation Deployment Stage Failed"
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
