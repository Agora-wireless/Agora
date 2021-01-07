def findLogFile() {
	jobName = "${env.JOB_NAME}"
	tokens = jobName.split('/')
	jobDir = tokens[0]
	filePath = "${env.JENKINS_HOME}/jobs/${jobDir}/branches/${env.JOB_BASE_NAME}/builds/${env.BUILD_NUMBER}/log"
	
	return filePath
}


def unitTest(log, utPF) {
	command = $/tail -10 ${log} | grep -i 'PASSED'/$
	pf_flag = sh(script: command, returnStdout: true)
	pf_flag = pf_flag.trim()
	if (pf_flag == utPF) {
		echo "Passing due to " + pf_flag
		currentBuild.result = "SUCCESS"
	} else {
		echo "Failing due to " + pf_flag
		currentBuild.result = "FAILURE"
	}
}


pipeline {
	agent any
	
	options {
		skipDefaultCheckout true
		buildDiscarder(logRotator(numToKeepStr:'9'))
	}
	
	
	stages {
		stage ("Start") {
			steps {
				echo "CI started ..."
				// slackSend (color: '#FFFF00', message: "Build STARTED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
			}
		}
		
		stage ("Repair Changelog") {
			when { expression { return !currentBuild.previousBuild } }
			steps {
				echo "CI preparation: Add Changelog to fix Jenkins' First-time-build bug ..."
				checkout([
					$class: 'GitSCM',
					branches: scm.branches,
					userRemoteConfigs: scm.userRemoteConfigs,
					browser: scm.browser,
					// build the changesets from the compareTarget branch
					extensions: [[$class: 'ChangelogToBranch', options: [compareRemote: 'origin', compareTarget: 'master']]]
				])
			}
		}
		
		// perform the normal configured checkout to ensure all configured extensions runs and to generate the changeset for later builds
		stage ("Checkout Source") {
			steps {
				echo "CI checking out from the source ..."
				checkout scm
			}
		}
		
		stage("Build Agora") {
			steps {
				echo "CI building Agora ..."
				dir("${WORKSPACE}") {
					sh '''
						source /opt/intel/compilers_and_libraries_2020.3.279/linux/bin/compilervars.sh intel64
						mkdir build && cd build
						cmake .. && make -j
					'''
				}
			}
		}
		
		stage("End-to-end Test") {
			steps {
				echo "CI end-to-end testing ..."
				dir("${WORKSPACE}") {
					sh '''
						export LD_LIBRARY_PATH=/opt/intel/compilers_and_libraries_2020.3.279/linux/mkl/lib/intel64/:$LD_LIBRARY_PATH
						./test/test_agora/test_agora.sh
					'''
					script {
						logFile = findLogFile()
						command = $/tail -300 ${logFile} | grep -i 'Passed uplink test!'/$
						ul_pf_flag = sh(script: command, returnStdout: true)
						ul_pf_flag = ul_pf_flag.trim()
						command = $/tail -300 ${logFile} | grep -i 'Passed downlink test!'/$
						dl_pf_flag = sh(script: command, returnStdout: true)
						dl_pf_flag = dl_pf_flag.trim()
						if (ul_pf_flag == "Passed uplink test!" && dl_pf_flag == "Passed downlink test!") {
							echo "Passing due to " + ul_pf_flag + " and " + dl_pf_flag
							currentBuild.result = "SUCCESS"
						} else {
							echo "Failing due to " + ul_pf_flag + " and " + dl_pf_flag
							currentBuild.result = "FAILURE"
						}
					}
				}
			}
		}
		
		stage("Unit Tests") {
			steps {
				echo "CI unit testing ..."
				dir("${WORKSPACE}") {
					echo "Testing test_datatype_conversion ..."
					sh '''
						export LD_LIBRARY_PATH=/opt/intel/compilers_and_libraries_2020.3.279/linux/mkl/lib/intel64/:$LD_LIBRARY_PATH
						./build/test_datatype_conversion
					'''
					script {
						unitTest(logFile, "[  PASSED  ] 3 tests.")
					}
					
					echo "Testing test_udp_client_server ..."
					sh '''
						export LD_LIBRARY_PATH=/opt/intel/compilers_and_libraries_2020.3.279/linux/mkl/lib/intel64/:$LD_LIBRARY_PATH
						./build/test_udp_client_server
					'''
					script {
						unitTest(logFile, "[  PASSED  ] 2 tests.")
					}
					
					echo "Testing test_concurrent_queue ..."
					sh '''
						export LD_LIBRARY_PATH=/opt/intel/compilers_and_libraries_2020.3.279/linux/mkl/lib/intel64/:$LD_LIBRARY_PATH
						./build/test_concurrent_queue
					'''
					script {
						unitTest(logFile, "[  PASSED  ] 3 tests.")
					}
					
					echo "Testing test_zf ..."
					sh '''
						export LD_LIBRARY_PATH=/opt/intel/compilers_and_libraries_2020.3.279/linux/mkl/lib/intel64/:$LD_LIBRARY_PATH
						./build/test_zf
					'''
					script {
						unitTest(logFile, "[  PASSED  ] 1 test.")
					}
					
					echo "Testing test_zf_threaded ..."
					sh '''
						export LD_LIBRARY_PATH=/opt/intel/compilers_and_libraries_2020.3.279/linux/mkl/lib/intel64/:$LD_LIBRARY_PATH
						./build/test_zf_threaded
					'''
					script {
						unitTest(logFile, "[  PASSED  ] 1 test.")
					}
					
					echo "Testing test_demul_threaded ..."
					sh '''
						export LD_LIBRARY_PATH=/opt/intel/compilers_and_libraries_2020.3.279/linux/mkl/lib/intel64/:$LD_LIBRARY_PATH
						./build/test_demul_threaded
					'''
					script {
						unitTest(logFile, "[  PASSED  ] 1 test.")
					}
					
					echo "Testing test_ptr_grid ..."
					sh '''
						export LD_LIBRARY_PATH=/opt/intel/compilers_and_libraries_2020.3.279/linux/mkl/lib/intel64/:$LD_LIBRARY_PATH
						./build/test_ptr_grid
					'''
					script {
						unitTest(logFile, "[  PASSED  ] 2 tests.")
					}
					
					echo "Testing test_recipcal ..."
					sh '''
						export LD_LIBRARY_PATH=/opt/intel/compilers_and_libraries_2020.3.279/linux/mkl/lib/intel64/:$LD_LIBRARY_PATH
						./build/test_recipcal
					'''
					script {
						unitTest(logFile, "[  PASSED  ] 1 test.")
					}
				}
			}
		}
	}
	
	
	post {
		success {
			echo "CI passed!"
			// slackSend (color: '#00FF00', message: "Build SUCCESSFUL: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
		}
		
		failure {
			echo "CI failed!"
			// slackSend (color: '#FF0000', message: "Build FAILED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
		}
	}
}


