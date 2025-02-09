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
                sh '''
                    docker run --rm \
                        -v ${WORKSPACE}:/workspace \
                        -w /workspace/ros_ws \
                        ros-jenkins:${BUILD_NUMBER} \
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
                    sh """
                        docker run --rm \
                            -v ${WORKSPACE}:/workspace \
                            -w /workspace/ros_ws \
                            ros-jenkins:\${BUILD_NUMBER} /bin/bash -c \
                            "source /opt/ros/noetic/setup.bash && \
                            source devel/setup.bash && \
                            mkdir -p build/test_results test_results && \
                            catkin run_tests --no-deps && \
                            catkin_test_results build/test_results --verbose > test_results/summary.txt && \
                            find build -type d -name test_results -exec cp -r {} /workspace/ros_ws/test_results/ \\; || true && \
                            find build -name '*.xml' -exec cp {} /workspace/ros_ws/test_results/ \\; || true"
                    """
                }
            }
            post {
                always {
                    junit allowEmptyResults: true, testResults: 'ros_ws/**/test_results/**/*.xml'
                    sh '''
                        docker run --rm \
                            -v ${WORKSPACE}:/workspace \
                            -w /workspace/ros_ws \
                            ros-jenkins:${BUILD_NUMBER} /bin/bash -c \
                            "python3 -m pytest src/agv_sim/test --html=test-report.html --self-contained-html"
                    '''
                    archiveArtifacts artifacts: 'ros_ws/test-report.html', allowEmptyArchive: true
                }
                success {
                    echo 'All tests passed!'
                }
            }
        }

        stage('Gazebo Simulation Deployment') {
            steps {
                script {
                    def simulationStatus = sh(
                        script: '''
                            set -e
                            echo "=== Starting Gazebo Simulation (with 10-minute timeout) ==="
                            
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
                                    timeout 600s roslaunch agv_sim simulation.launch \
                                        use_rviz:=false \
                                        gui:=false \
                                        record:=true \
                                        --screen \
                                        --wait"
                        ''',
                        returnStatus: true
                    )

                    if (simulationStatus == 124) {
                        echo "Simulation reached the timeout. Stopping gracefully..."
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
                    // Archive test results and generate reports
                    junit '**/test_results/**/*.xml'
                    
                    // Generate and archive detailed HTML test report
                    sh '''
                        python3 -m pytest ros_ws/src/agv_sim/test --html=test-report.html --self-contained-html
                        python3-coverage run -m pytest ros_ws/src/agv_sim/test
                        python3-coverage html -d coverage-report
                    '''
                    
                    // Archive test and coverage reports
                    archiveArtifacts artifacts: 'test-report.html,coverage-report/**/*', allowEmptyArchive: true
                    
                    // Publish HTML reports
                    publishHTML(target: [
                        allowMissing: true,
                        alwaysLinkToLastBuild: true,
                        keepAll: true,
                        reportDir: '.',
                        reportFiles: 'test-report.html',
                        reportName: 'Test Report'
                    ])
                    
                    publishHTML(target: [
                        allowMissing: true,
                        alwaysLinkToLastBuild: true,
                        keepAll: true,
                        reportDir: 'coverage-report',
                        reportFiles: 'index.html',
                        reportName: 'Coverage Report'
                    ])
                }
            }
        }

        stage('Documentation Archiving') {
            steps {
                script {
                    // Generate and archive documentation
                    sh '''
                        # Install asciidoctor-pdf if not present
                        if ! command -v asciidoctor-pdf &> /dev/null; then
                            gem install asciidoctor-pdf
                        fi
                        
                        # Create documentation directory
                        mkdir -p documentation
                        
                        # Generate documentation
                        cat << EOF > documentation/pipeline.adoc
                        = AGV Simulation Pipeline Documentation
                        :doctype: book
                        :toc:
                        
                        == Pipeline Overview
                        This pipeline automates the build, test, and deployment process for the AGV simulation project.
                        
                        == Stages
                        
                        === 1. Build Stage
                        * Builds ROS workspace using catkin
                        * Compiles all packages and dependencies
                        
                        === 2. Test Stage
                        * Runs unit tests using pytest
                        * Executes integration tests
                        * Generates test coverage reports
                        
                        === 3. Simulation Stage
                        * Launches Gazebo simulation environment
                        * Runs AGV simulation tests
                        * Records simulation metrics and logs
                        
                        === 4. Deploy Stage
                        * Deploys successful builds
                        * Archives artifacts and reports
                        
                        == Gazebo Integration
                        The pipeline integrates with Gazebo simulator through ROS:
                        
                        * Uses Docker container with ROS and Gazebo
                        * Configures virtual display for headless execution
                        * Manages simulation timeouts and cleanup
                        
                        == Reports and Metrics
                        * Test Results: JUnit XML and HTML reports
                        * Coverage: Python coverage reports
                        * Simulation Logs: Archived for analysis
                        EOF
                        
                        # Convert to PDF
                        asciidoctor-pdf documentation/pipeline.adoc -o documentation/pipeline.pdf
                    '''
                    
                    // Archive documentation
                    archiveArtifacts artifacts: 'documentation/*.pdf', allowEmptyArchive: true
                }
            }
        }

        stage('Dashboard Insights') {
            steps {
                script {
                    // Clean up
                    cleanWs()
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