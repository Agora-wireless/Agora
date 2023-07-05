import subprocess

def run_script(script_name):
    subprocess.run(["python3", script_name])

if __name__ == "__main__":
    script1 = "cubic_mimo.py"
    script2 = "cubic_conste.py"
    script3 = "cubic_timeD.py"

    process1 = subprocess.Popen(["python3", script1])
    process2 = subprocess.Popen(["python3", script2])
    process3 = subprocess.Popen(["python3", script3])

    process1.wait()
    process2.wait()
    process3.wait()
