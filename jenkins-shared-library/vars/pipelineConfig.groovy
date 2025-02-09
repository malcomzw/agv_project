#!/usr/bin/env groovy

def getConfig() {
    return [
        dockerImage: 'ros-jenkins',
        timeouts: [
            build: 10,
            test: 5,
            simulation: 5
        ],
        ros: [
            version: 'noetic',
            masterUri: 'http://localhost:11311'
        ]
    ]
}
