pipeline {
    agent {
        docker {
            image 'osrf/ros:noetic-desktop-full'
            args '-e ROS_DISTRO=noetic'
        }
    }
    
    environment {
        CATKIN_WS = '/workspace/ros_ws'
    }
    
    stages {
        stage('Prepare') {
            steps {
                sh '''
                    mkdir -p $CATKIN_WS/src
                    cd $CATKIN_WS
                    ln -s /workspace/src/* src/
                    rosdep init
                    rosdep update
                    rosdep install --from-paths src --ignore-src -r -y
                '''
            }
        }
        
        stage('Build') {
            steps {
                sh '''
                    cd $CATKIN_WS
                    source /opt/ros/noetic/setup.bash
                    catkin_make tests
                    catkin_make
                '''
            }
        }
        
        stage('Test') {
            steps {
                sh '''
                    cd $CATKIN_WS
                    source devel/setup.bash
                    catkin_test_results build/test_results || true
                '''
                junit 'build/test_results/**/*.xml'
            }
        }
        
        stage('Simulation') {
            steps {
                sh '''
                    cd $CATKIN_WS
                    source devel/setup.bash
                    rostest agv_sim simulation_test.test
                '''
            }
        }
    }
    
    post {
        always {
            archiveArtifacts artifacts: 'build/test_results/**/*.xml', fingerprint: true
        }
    }
}
