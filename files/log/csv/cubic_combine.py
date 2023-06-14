import subprocess

def run_script(script_name):
    subprocess.run(["python3", script_name])

if __name__ == "__main__":
    script1 = "cubic.py"
    script2 = "cubic_conste.py"

    process1 = subprocess.Popen(["python3", script1])
    process2 = subprocess.Popen(["python3", script2])

    process1.wait()
    process2.wait()
