
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
		lock resource: 'shared_resource_lock'
	}
	
	
	stages {
		stage ("Start") {
			steps {
				echo "CI started ..."
				slackSend (color: '#FFFF00', 
						   message: "Build STARTED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
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
						. /opt/intel/oneapi/setvars.sh --config="/opt/intel/oneapi/renew-config.txt"
						mkdir build && cd build
						cmake .. && make -j && cd ..
						cp ./build/CTestTestfile.cmake ./
					'''
				}
			}
		}
		
		stage("Test Agora") {
			steps {
				echo "CI Emulated Agora testing ..."
				dir("${WORKSPACE}") {
					sh '''
						. /opt/intel/oneapi/setvars.sh --config="/opt/intel/oneapi/renew-config.txt"
						./test/test_agora/test_agora.sh
					'''
					script {
						logFile = findLogFile()
						command = $/tail -1500 ${logFile} | grep -i 'Passed uplink test!'/$
						ul_pf_flag = sh(script: command, returnStdout: true)
						ul_pf_flag = ul_pf_flag.trim()
						command = $/tail -1500 ${logFile} | grep -i 'Passed downlink test!'/$
						dl_pf_flag = sh(script: command, returnStdout: true)
						dl_pf_flag = dl_pf_flag.trim()
						command = $/tail -1500 ${logFile} | grep -i 'Passed combined test!'/$
						comb_pf_flag = sh(script: command, returnStdout: true)
						comb_pf_flag = comb_pf_flag.trim()
						if (ul_pf_flag == "Passed uplink test!" && dl_pf_flag == "Passed downlink test!" && comb_pf_flag == "Passed combined test!") {
							echo "Passing due to " + ul_pf_flag + " and " + dl_pf_flag + " and " + comb_pf_flag
							currentBuild.result = "SUCCESS"
						} else {
							echo "Failing due to " + ul_pf_flag + " and " + dl_pf_flag + " and " + comb_pf_flag
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
					echo "Building data for unit tests ..."
					sh '''
						. /opt/intel/oneapi/setvars.sh --config="/opt/intel/oneapi/renew-config.txt"
						./build/data_generator --conf_file files/config/ci/tddconfig-sim-ul.json
					'''
					
					sh '''
						. /opt/intel/oneapi/setvars.sh --config="/opt/intel/oneapi/renew-config.txt"
						ctest
					'''
				}
			}
		}
		
		stage("End-to-end Test with Channel Simulator") {
			steps {
				echo "CI E2E testing ..."
				dir("${WORKSPACE}") {
					sh '''
						. /opt/intel/oneapi/setvars.sh --config="/opt/intel/oneapi/renew-config.txt"
						./test/sim_tests/test_e2e_sim.sh 0.05
					'''
					script {
						logFile = findLogFile()
						command = $/tail -300 ${logFile} | grep -i 'Passed the end-to-end test with channel simulator!'/$
						e2e_pf_flag = sh(script: command, returnStdout: true)
						e2e_pf_flag = e2e_pf_flag.trim()
						if (e2e_pf_flag == "Passed the end-to-end test with channel simulator!") {
							echo "Passing due to " + e2e_pf_flag
							currentBuild.result = "SUCCESS"
						} else {
							echo "Failing due to " + e2e_pf_flag
							currentBuild.result = "FAILURE"
						}
					}
				}
			}
		}
	}
	
	
	post {
		success {
			echo "CI passed!"
			slackSend (color: '#00FF00', message: "Build SUCCESSFUL: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
		}
		
		failure {
			echo "CI failed!"
			slackSend (color: '#FF0000', message: "Build FAILED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
		}
	}
}