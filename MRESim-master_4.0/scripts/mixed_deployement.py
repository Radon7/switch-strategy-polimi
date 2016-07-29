import subprocess
import time
import os

if __name__ == "__main__":
    mydir = "" #"/home/banfi/NetBeansProjects/MRESim-master_4.0/"
    fwin = open(mydir + "winner.txt", "w")
    pIlp = subprocess.Popen(["python", mydir + "scripts/ilp_model.py", "mix"])
    pApp = subprocess.Popen(["python", mydir + "scripts/approximate_deployement.py", "mix"])
    start = time.time()
    while(time.time() - start < 10.0):
        time.sleep(2)
        filp = open(mydir + "mixIlp.txt", "r")
        line = filp.readlines()[0]
        filp.close()
        if(line == "DONE"):
            pIlp.kill()
            pApp.kill()
            fwin.write("1")
            fwin.close()
            os.system("mv " + mydir + "currentTreeIlp.txt " + mydir + "currentTree.txt")
            os.system("mv " + mydir + "waitingIlp.txt " + mydir + "waiting.txt")
            os.system("mv " + mydir + "m_optIlp.txt " + mydir + "m_opt.txt")
            os.system("mv " + mydir + "costMinIlp.txt " + mydir + "costMin.txt")
            exit(0)


    while(time.time() - start < 1200):
        time.sleep(2)
        filp = open(mydir + "mixIlp.txt", "r")
        line = filp.readlines()[0]
        filp.close()
        if(line == "DONE"):
            pIlp.kill()
            pApp.kill()
            fwin.write("1")
            fwin.close()
            os.system("mv " + mydir + "currentTreeIlp.txt " + mydir + "currentTree.txt")
            os.system("mv " + mydir + "waitingIlp.txt " + mydir + "waiting.txt")
            os.system("mv " + mydir + "m_optIlp.txt " + mydir + "m_opt.txt")
            os.system("mv " + mydir + "costMinIlp.txt " + mydir + "costMin.txt")
            exit(0)

        fapp = open(mydir + "mixApp.txt", "r")
        line = fapp.readlines()[0]
        fapp.close()
        if(line == "DONE"):
            pIlp.kill()
            pApp.kill()
            fwin.write("2")
            fwin.close()
            os.system("mv " + mydir + "currentTreeApp.txt " + mydir + "currentTree.txt")
            os.system("mv " + mydir + "waitingApp.txt " + mydir + "waiting.txt")
            os.system("mv " + mydir + "m_optApp.txt " + mydir + "m_opt.txt")
            os.system("mv " + mydir + "costMinApp.txt " + mydir + "costMin.txt")
            exit(0)

        
