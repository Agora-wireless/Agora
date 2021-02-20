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
						source /opt/intel/compilers_and_libraries_2020.3.279/linux/bin/compilervars.sh intel64
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
						export LD_LIBRARY_PATH=/opt/intel/compilers_and_libraries_2020.3.279/linux/mkl/lib/intel64/:$LD_LIBRARY_PATH
						./test/test_agora/test_agora.sh
					'''
				}
			}
		}
		
		stage("Unit Tests") {
			steps {
				echo "CI unit testing ..."
				dir("${WORKSPACE}") {
					echo "Building data for unit tests ..."
					sh '''
						export LD_LIBRARY_PATH=/opt/intel/compilers_and_libraries_2020.3.279/linux/mkl/lib/intel64/:$LD_LIBRARY_PATH
						./build/data_generator --conf_file data/tddconfig-sim-ul.json
					'''
					
					sh '''
						export LD_LIBRARY_PATH=/opt/intel/compilers_and_libraries_2020.3.279/linux/mkl/lib/intel64/:$LD_LIBRARY_PATH
						ctest
					'''
				}
			}
		}
		
		stage("End-to-end Test with Channel Simulator") {
			steps {
				echo "CI e2e testing ..."
				dir("${WORKSPACE}") {
					sh '''
						export LD_LIBRARY_PATH=/opt/intel/compilers_and_libraries_2020.3.279/linux/mkl/lib/intel64/:$LD_LIBRARY_PATH
						./test/sim_tests/test_e2e_sim.sh 0.001
					'''
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
