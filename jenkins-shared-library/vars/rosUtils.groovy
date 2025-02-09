#!/usr/bin/env groovy

def buildRosPackage(Map config = [:]) {
    def dockerImage = config.dockerImage ?: 'ros-jenkins'
    def buildNumber = config.buildNumber ?: env.BUILD_NUMBER
    
    sh """
        docker run --rm \
            -v \${WORKSPACE}:/workspace \
            -w /workspace/ros_ws \
            ${dockerImage}:\${buildNumber} \
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
    """
}

def runRosTests(Map config = [:]) {
    def dockerImage = config.dockerImage ?: 'ros-jenkins'
    def buildNumber = config.buildNumber ?: env.BUILD_NUMBER
    
    sh """
        docker run --rm -v \${WORKSPACE}:/workspace -w /workspace/ros_ws ${dockerImage}:\${buildNumber} /bin/bash -c "
            source /opt/ros/noetic/setup.bash &&
            source devel/setup.bash &&
            mkdir -p build/test_results test_results &&
            catkin run_tests --no-deps &&
            catkin_test_results build/test_results --verbose > test_results/summary.txt &&
            find build -type d -name test_results -exec cp -r {} /workspace/ros_ws/test_results/ \\; || true &&
            find build -name '*.xml' -exec cp {} /workspace/ros_ws/test_results/ \\; || true"
    """
}

def runGazeboSimulation(Map config = [:]) {
    def dockerImage = config.dockerImage ?: 'ros-jenkins'
    def buildNumber = config.buildNumber ?: env.BUILD_NUMBER
    def timeout = config.timeout ?: 300
    
    sh """
        docker run --rm \
            -v \${WORKSPACE}:/workspace \
            -w /workspace/ros_ws \
            --env DISPLAY=:99 \
            --env LIBGL_ALWAYS_SOFTWARE=1 \
            --env ROS_MASTER_URI=http://localhost:11311 \
            --env ROS_HOSTNAME=localhost \
            ${dockerImage}:\${buildNumber} bash -c "
                source /opt/ros/noetic/setup.bash && \
                catkin build && \
                source devel/setup.bash && \
                timeout ${timeout}s roslaunch agv_sim simulation.launch \
                    use_rviz:=false \
                    gui:=false \
                    record:=true \
                    --screen \
                    --wait"
    """
}
