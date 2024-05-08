#!groovy

def cronSettings = "0 20 * * *"
env.branch = ""
env.commitid = ""


pipeline {
    agent {
        label "linux_node"
    }    // agent

    options {
        skipDefaultCheckout()
        buildDiscarder(logRotator(numToKeepStr: '15', artifactNumToKeepStr: '15'))
        timestamps()
    }

    triggers {
        cron(cronSettings)
    }

    stages {
        stage("Checkout repo"){
            when {
                branch BRANCH_NAME
            }

            steps {
                deleteDir()
                checkout scm
                script {
                    env.commitid = sh(returnStdout: true, script: 'git rev-parse --short HEAD').trim()
                    env.branch = BRANCH_NAME
                }
            }
        }

        stage("Trigger Downstream Job"){
            when {
                branch BRANCH_NAME
            }

            parallel {
                stage("zephyr_linux_build") {
                    steps{
                        build(job: "zephyr_linux", propagate: true, wait: true)
                    }
                }
                stage("zephyr_windows_build") {
                    steps{
                        build(job: "zephyr_windows", propagate: true, wait: true)
                    }
                }
            }
        }

    }

}// pipeline

