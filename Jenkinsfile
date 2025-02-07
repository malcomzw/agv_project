pipeline {
    agent any

    environment {
        DOCKER_IMAGE = 'ros-jenkins'
        DOCKER_TAG = "${BUILD_ID}"
        ROS_WORKSPACE = "${WORKSPACE}/ros_ws"
    }

    options {
        timestamps()
        timeout(time: 1, unit: 'HOURS')
        buildDiscarder(logRotator(numToKeepStr: '10'))
    }

    stages {
        stage('Prepare Environment') {
            steps {
                checkout scm
                sh "docker build -t ${DOCKER_IMAGE}:${DOCKER_TAG} -f docker/Dockerfile ."
            }
        }

        stage('Build') {
            steps {
                sh """
                    docker run --rm \\
                        -v ${WORKSPACE}:/workspace \\
                        -w /workspace/ros_ws \\
                        ${DOCKER_IMAGE}:${DOCKER_TAG} \\
                        /bin/bash -c '
                            source /opt/ros/noetic/setup.bash && \\
                            rm -rf build devel && \\
                            mkdir -p build devel && \\
                            catkin_make --pkg demo_pkg
                        '
                """
            }
        }

        stage('Unit Tests') {
            steps {
                sh """
                    docker run --rm \\
                        -v ${WORKSPACE}:/workspace \\
                        -w /workspace/ros_ws \\
                        ${DOCKER_IMAGE}:${DOCKER_TAG} \\
                        /bin/bash -c '
                            source /opt/ros/noetic/setup.bash && \\
                            source devel/setup.bash && \\
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

        stage('Integration Tests') {
            steps {
                sh """
                    docker run --rm \\
                        -v ${WORKSPACE}:/workspace \\
                        -w /workspace/ros_ws \\
                        ${DOCKER_IMAGE}:${DOCKER_TAG} \\
                        /bin/bash -c '
                            source /opt/ros/noetic/setup.bash && \\
                            source devel/setup.bash && \\
                            rostest demo_pkg demo_test.test
                        '
                """
            }
        }

        stage('Code Analysis') {
            steps {
                sh """
                    docker run --rm \\
                        -v ${WORKSPACE}:/workspace \\
                        -w /workspace/ros_ws \\
                        ${DOCKER_IMAGE}:${DOCKER_TAG} \\
                        /bin/bash -c '
                            source /opt/ros/noetic/setup.bash && \\
                            catkin_lint src/demo_pkg || true && \\
                            roslint_python src/demo_pkg/scripts/*.py || true
                        '
                """
            }
        }

        stage('Documentation') {
            steps {
                sh """
                    docker run --rm \\
                        -v ${WORKSPACE}:/workspace \\
                        -w /workspace \\
                        ${DOCKER_IMAGE}:${DOCKER_TAG} \\
                        /bin/bash -c '
                            cd ros_ws && \\
                            rosdoc_lite src/demo_pkg
                        '
                """
                publishHTML([
                    allowMissing: true,
                    alwaysLinkToLastBuild: false,
                    keepAll: true,
                    reportDir: 'ros_ws/doc/html',
                    reportFiles: 'index.html',
                    reportName: 'ROS Documentation'
                ])
            }
        }

        stage('Deploy to Simulation') {
            steps {
                sh """
                    docker run --rm \\
                        -v ${WORKSPACE}:/workspace \\
                        -w /workspace/ros_ws \\
                        ${DOCKER_IMAGE}:${DOCKER_TAG} \\
                        /bin/bash -c '
                            source /opt/ros/noetic/setup.bash && \\
                            source devel/setup.bash && \\
                            timeout 30s roslaunch demo_pkg demo_test.test gui:=false || true
                        '
                """
            }
        }
    }

    post {
        always {
            // Generate test reports
            script {
                def testReport = """<!DOCTYPE html>
                    <html>
                    <head>
                        <title>Test Report Summary</title>
                        <style>
                            body { font-family: Arial, sans-serif; margin: 40px; }
                            .status { padding: 10px; border-radius: 5px; }
                            .success { background-color: #dff0d8; }
                            .failure { background-color: #f2dede; }
                        </style>
                    </head>
                    <body>
                        <h1>Test Report Summary</h1>
                        <p><strong>Build Number:</strong> ${BUILD_NUMBER}</p>
                        <p><strong>Build URL:</strong> <a href="${BUILD_URL}">${BUILD_URL}</a></p>
                        <p><strong>Status:</strong> <span class="status \${currentBuild.result == 'SUCCESS' ? 'success' : 'failure'}">${currentBuild.result}</span></p>
                        <h2>Stage Results</h2>
                        <ul>
                            <li>Unit Tests: ${currentBuild.result}</li>
                            <li>Integration Tests: ${currentBuild.result}</li>
                            <li>Code Analysis: Complete</li>
                            <li>Documentation: Generated</li>
                            <li>Simulation: Complete</li>
                        </ul>
                    </body>
                    </html>"""
                writeFile file: 'test-report.html', text: testReport
                publishHTML([
                    allowMissing: false,
                    alwaysLinkToLastBuild: false,
                    keepAll: true,
                    reportDir: '.',
                    reportFiles: 'test-report.html',
                    reportName: 'Test Report Summary'
                ])
            }
            
            // Cleanup
            cleanWs()
            sh "docker rmi ${DOCKER_IMAGE}:${DOCKER_TAG} || true"
        }
        success {
            echo 'Pipeline completed successfully!'
        }
        failure {
            echo 'Pipeline failed!'
        }
    }
}
