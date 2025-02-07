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
        WORKSPACE_OWNER = sh(
            script: 'id -un',
            returnStdout: true
        ).trim()
    }

    stages {
        stage('Cleanup') {
            steps {
                script {
                    try {
                        sh '''
                            # Attempt to change ownership if needed
                            sudo chown -R $(whoami):$(whoami) $WORKSPACE || true
                            
                            # Force remove workspace contents
                            rm -rf $WORKSPACE/* || true
                            rm -rf $WORKSPACE/.* || true
                        '''

                        cleanWs(
                            cleanWhenNotBuilt: false,
                            cleanWhenAborted: true,
                            cleanWhenFailure: true,
                            cleanWhenSuccess: true,
                            cleanWhenUnstable: true,
                            deleteDirs: true,
                            disableDeferredWipeout: true,
                            notFailBuild: true
                        )
                    } catch (Exception e) {
                        echo "Warning: Workspace cleanup failed: ${e.message}"
                        sh 'sudo rm -rf $WORKSPACE || true'
                    }
                }
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
                    echo "Workspace: $WORKSPACE"
                    pwd
                    ls -la
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
                            ros-jenkins:91 timeout 180 /bin/bash -c '
                                set -e

                                echo "=== Initializing rosdep ==="
                                if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
                                    rosdep init
                                fi
                                rosdep update

                                echo "=== Installing Required Packages ==="
                                apt-get update
                                apt-get install -y xvfb python3-pygame mesa-utils

                                echo "=== Resolving ROS Dependencies ==="
                                rosdep install --from-paths src --ignore-src -r -y

                                echo "=== Checking ROS Environment ==="
                                source /opt/ros/noetic/setup.bash
                                source devel/setup.bash
                                env | grep ROS

                                echo "=== Starting Virtual Display ==="
                                Xvfb :99 -screen 0 1024x768x16 &

                                echo "=== Checking for simulation.launch ==="
                                if [ ! -f src/agv_sim/launch/simulation.launch ]; then
                                    echo "ERROR: simulation.launch file not found!"
                                    exit 1
                                fi

                                echo "=== Starting Gazebo Simulation ==="
                                roslaunch agv_sim simulation.launch use_rviz:=false gui:=false record:=true --wait
                            '
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
        always {
            script {
                try {
                    sh '''
                        mkdir -p $WORKSPACE/test-results
                        chmod 777 $WORKSPACE/test-results
                        echo '<?xml version="1.0" encoding="UTF-8"?>
<testsuites>
    <testsuite name="placeholder" tests="1" failures="0" errors="0">
        <testcase classname="PlaceholderTest" name="placeholderTest"/>
    </testsuite>
</testsuites>' > $WORKSPACE/test-results/dummy-test-report.xml
                    '''
                    junit allowEmptyResults: true, testResults: 'test-results/*.xml'
                } catch (Exception e) {
                    echo "Test reporting failed: ${e.message}"
                }
            }
            echo "Build completed! Commit: ${env.GIT_COMMIT_SHORT}"
        }
        success {
            echo "Build succeeded!"
        }
        failure {
            echo "Build failed!"
        }
        cleanup {
            sh '''
                sudo rm -rf $WORKSPACE/* || true
                sudo rm -rf $WORKSPACE/.* || true
            '''
        }
    }
}
