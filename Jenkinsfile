// Configure the Jenkins shared library
library identifier: 'jenkins-ros-library@main',
        retriever: modernSCM([
            $class: 'GitSCMSource',
            remote: 'https://github.com/malcomzw/jenkins-ros-library.git'
        ])

pipeline {
    agent any

    options {
        timestamps()
        timeout(time: 10, unit: 'MINUTES')
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
                sh 'docker build -t ros-jenkins:${BUILD_NUMBER} -f docker/Dockerfile .'
            }
        }

        stage('Build ROS Package') {
            steps {
                script {
                    def config = pipelineConfig.getConfig()
                    rosUtils.buildRosPackage(
                        dockerImage: config.dockerImage,
                        buildNumber: env.BUILD_NUMBER
                    )
                }
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
                    // Run ROS tests
                    sh '''
                        docker run --rm -v ${WORKSPACE}:/workspace -w /workspace/ros_ws ros-jenkins:${BUILD_ID} /bin/bash -c "
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
                    junit '**/test_results/**/*.xml'
                }
                success {
                    echo 'All tests passed!'
                }
                failure {
                    echo 'Tests failed!'
                }
            }
        }

        stage('Gazebo Simulation Deployment') {
            steps {
                script {
                    def simulationStatus = sh(
                        script: '''
                            set -e
                            echo "=== Starting Gazebo Simulation (with 5-minute timeout) ==="
                            
                            docker run --rm \
                                -v ${WORKSPACE}:/workspace \
                                -w /workspace/ros_ws \
                                --env DISPLAY=:99 \
                                --env LIBGL_ALWAYS_SOFTWARE=1 \
                                --env ROS_MASTER_URI=http://localhost:11311 \
                                --env ROS_HOSTNAME=localhost \
                                ros-jenkins:${BUILD_NUMBER} bash -c "\
                                    source /opt/ros/noetic/setup.bash && \
                                    catkin build && \
                                    source devel/setup.bash && \
                                    
                                    # Run simulation with timeout
                                    timeout 300s roslaunch agv_sim simulation.launch \
                                        use_rviz:=false \
                                        gui:=false \
                                        record:=true \
                                        --screen \
                                        --wait"
                        ''',
                        returnStatus: true
                    )

                    if (simulationStatus == 124) {
                        echo "Simulation reached the expected 5-minute duration. Stopping gracefully..."
                        return 0
                    } else if (simulationStatus != 0) {
                        error "Simulation failed with exit code ${simulationStatus}"
                    }
                }
            }
            post {
                success {
                    echo "Gazebo Simulation Deployment Passed (Completed or Stopped at Timeout)"
                }
                failure {
                    error "Gazebo Simulation Deployment Failed"
                }
                always {
                    sh 'docker ps -q --filter "ancestor=ros-jenkins:${BUILD_NUMBER}" | xargs -r docker stop'
                }
            }
        }

        stage('Detailed Report Generation') {
            steps {
                script {
                    // Archive test results
                    junit allowEmptyResults: true, testResults: '**/test_results/**/*.xml'
                    
                    // Generate HTML test report in Docker
                    sh '''
                        docker run --rm \
                            -v ${WORKSPACE}:/workspace \
                            -w /workspace/ros_ws \
                            ros-jenkins:${BUILD_ID} /bin/bash -c \
                            "source /opt/ros/noetic/setup.bash && \
                             source devel/setup.bash && \
                             mkdir -p test_results && \
                             python3 -m pytest src/agv_sim/test --html=test_results/test-report.html --self-contained-html || true"
                    '''
                    
                    // Archive test report
                    archiveArtifacts artifacts: 'ros_ws/test_results/test-report.html', allowEmptyArchive: true
                    
                    // Publish HTML report
                    publishHTML(target: [
                        allowMissing: true,
                        alwaysLinkToLastBuild: true,
                        keepAll: true,
                        reportDir: 'ros_ws/test_results',
                        reportFiles: 'test-report.html',
                        reportName: 'Test Report'
                    ])
                }
            }
        }

        stage('Documentation') {
            steps {
                script {
                    // Archive documentation
                    archiveArtifacts artifacts: 'docs/jenkins_pipeline_documentation.pdf', allowEmptyArchive: true
                }
            }
        }

        stage('Dashboard Insights') {
            steps {
                script {
                    // Simple cleanup
                    cleanWs(patterns: [[pattern: 'documentation', type: 'INCLUDE']])
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
            junit 'ros_ws/**/test_results/*.xml'
            cleanWs()
        }
    }
}
