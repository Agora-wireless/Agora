import os
import subprocess
import time
import pandas as pd
import numpy
from multiprocessing import active_children
import signal



def parseSweepParams(parameterFile):
        '''
        Args:

        Returns:

        '''
        return



def buildConfigFile(parameters):
                return


def run_simulation():
        
        pid = os.fork()

        if (pid == 0): #first child proccess. Sender

                path = os.getcwd()

                print("CWD: " + path)

                path = path + "/Agora"
                
                os.chdir(path)

                print("New CWD: ")

                print(os.getcwd())


                genOutput = open("swpeerTesting/genOutput.txt", "w")
                genError = open("swpeerTesting/genError.txt", "w")
                

                gen = subprocess.run(["./build/data_generator", "--conf_file", "files/config/examples/dl-sim.json"], stdout=genOutput, stderr=genError, text=True)
                print("Printing result of running data generator in a subprocess")
                genOutput.close
                genError.close

                print("sleeping")
                time.sleep(4)
                print("slept for 4 seconds")

                sendOutput = open("swpeerTesting/sendOutput.txt", "w")
                sendError = open("swpeerTesting/sendError.txt", "w")

                send = subprocess.run(["./build/sender", "--conf_file", "files/config/examples/dl-sim.json"], stdout=True, stderr=True, text=True)
                #print(send)
                sendOutput.close
                sendError.close

                time.sleep(10)

                os.kill(os.getpid, signal.SIGINT)
               

        else: #Create the second child process
                pid2 = os.fork()

                if(pid2 == 0): #second child process. Agora

                        path = os.getcwd()

                        print("CWD: " + path)

                        path = path + "/Agora"
                
                        os.chdir(path)

                        print("New CWD: ")

                        print(os.getcwd())
                        print("Agora is sleeping")
                        time.sleep(2)
                        print("Agora is waking up")

                        agoraOutput = open("swpeerTesting/agoraOutput.txt", "w")
                        agoraError = open("swpeerTesting/agoraError.txt", "w")

                        agora = subprocess.run(["./build/agora", "--conf_file", "files/config/examples/dl-sim.json"], stdout=True, stderr=True, text=True)
                        #print(agora)

                        agoraError.close()
                        agoraOutput.close()

                        time.sleep(10)

                        os.kill(os.getpid(), signal.SIGINT)
                        
                        
        #Subprocess.run() might reap itself but that
        #still wouldn't account for the wto forks I did.
        try:
                while (os.wait()[1] > 0):
                        pass
        except ChildProcessError:
                print("All children are reaped")  
        
        time.sleep(10)

        print("CWD: " + os.getcwd())

        genFile = open("/space/noah/Agora/swpeerTesting/genOutput.txt", "r")
        senderFile = open("/space/noah/Agora/swpeerTesting/genOutput.txt", "r")
        agoraFile = open("/space/noah/Agora/swpeerTesting/genOutput.txt", "r")

        data = genFile.readlines
        senderStats = senderFile.readlines()
        agoraStats = agoraFile.readlines()

        genFile.close()
        senderFile.close()
        agoraFile.close()

        df_entry = {
                "Trial": "1",
                "Generator": data,
                "Sender": senderStats,
                "Agora": agoraStats
        }

        df = pd.DataFrame(df_entry)

        print(df)
                        
        return


print("Testing Begining")
run_simulation()
print("Testing Ending")






