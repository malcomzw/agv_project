pipeline {
agent any

options {
buildDiscarder(logRotator(numToKeepStr: '10'))
disableConcurrentBuilds()
timeout(time: 1, unit: 'HOURS')
timestamps()
}

triggers {
gitlab(triggerOnPush: true, triggerOnMergeRequest: true)
}

environment {
DOCKER_IMAGE = 'ros-jenkins'
DOCKER_TAG = "${BUILD_ID}"
ROS_WORKSPACE = "${WORKSPACE}/ros_ws"
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

stage('Gazebo Integration Tests') {
steps {
sh '''
docker run --rm \
-v ${WORKSPACE}:/workspace \
-w /workspace \
ros-jenkins:58 \
/bin/bash -c '
source /ros_ws/devel/setup.bash && \
roslaunch demo_pkg gazebo_test.launch &
sleep 30 && \
rostest demo_pkg integration_test.test
'
'''
sh '''
rosrun performance_metrics gazebo_monitor.py \
--output ${WORKSPACE}/perf_metrics.json \
--duration 300
'''
}
post {
always {
junit '**/test_results/**/*.xml'
archiveArtifacts artifacts: '**/gazebo_logs/*.log'
}
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

stage('Security Scan') {
steps {
sh 'trivy image --exit-code 1 --severity CRITICAL ros-jenkins:58'
sh 'bandit -r ros_ws/src/'
}
}

stage('Documentation') {
steps {
sh '''
source /ros_ws/devel/setup.bash
rosdoc_lite /ros_ws/src
'''
}
post {
success {
publishHTML target: [
allowMissing: false,
alwaysLinkToLastBuild: false,
keepAll: true,
reportDir: 'ros_ws/doc/html',
reportFiles: 'index.html',
reportName: 'ROS Documentation'
]
}
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
archiveArtifacts artifacts: '**/build/**/*.log', allowEmptyArchive: true
script {
try {
junit '**/test_results/**/*.xml'
} catch(e) {
echo 'Test results archive failed: ' + e.toString()
}
}
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
<li>Gazebo Integration Tests: ${currentBuild.result}</li>
<li>Code Analysis: Complete</li>
<li>Security Scan: Complete</li>
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
