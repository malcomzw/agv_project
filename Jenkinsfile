pipeline {
    agent any

    options {
        timestamps()
        timeout(time: 5, unit: 'MINUTES')
        skipDefaultCheckout()
        disableConcurrentBuilds()
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
                sh '''
                    git rev-parse HEAD > .git-commit
                    echo "Build from commit: $(cat .git-commit)"
                    echo "On branch: $(git rev-parse --abbrev-ref HEAD)"
                    git show -s --format=%B HEAD
                '''
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
    }

    post {
        success {
            echo "Build succeeded! Commit: $(cat .git-commit)"
        }
        failure {
            echo "Build failed! Commit: $(cat .git-commit)"
        }
        always {
            echo 'Pipeline completed.'
            cleanWs()
        }
    }
}
