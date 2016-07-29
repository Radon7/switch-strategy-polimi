import os

def compile_config(method, nr, seed):
    fold = open("config/config_base.txt","r")
    fnew = open("config/lastSimulatorConfig.txt", "w")
    lines = fold.readlines()
    fold.close()
    for i in range(len(lines)):
        line = lines[i]
        if i == 0:
            fnew.write(method[0] + "\n")
        elif i == 1:
            if(method[0]=="FrontierExploration"):
                fnew.write("UtilReturn" + "\n")
            else:
                fnew.write("ReturnWhenComplete" + "\n")
        elif i == 19:
            fnew.write(str(seed) + "\n")
        elif i == 18:
            #if method[0] == "ApproxExploration" or method[0] == "IlpExploration":
            fnew.write(nr + "\n")

        elif i == 20:
            #approx guarantee
            fnew.write("4\n")
        elif i == 23:
            if(method[0]=="FrontierExploration"):
                fnew.write("true\n")
            else:
                fnew.write("false\n")
        elif i == 24:
            fnew.write("900\n")
        else:
            fnew.write(line)
            
    fnew.close()            
        

if __name__ == "__main__":
    environments = ["grass","open","offices"]
    num_robot = ["12"]
    #thresholds = {}
    #thresholds["12"] = ["12"]
    #num_robot = ["8"]
    #explMethods = [("FrontierExploration","0.9"),("FrontierExploration","0.5"), ("FrontierExploration","0.1")]
    explMethods = [("FrontierExploration", None)]    
    num_run = 5 #for different seeds and team configs

    os.system("rm -R logs_batch")
    os.system("mkdir logs_batch")

    os.system("rm -R logs")
    os.system("mkdir logs")

    os.system("rm -R log_file")
    os.system("mkdir log_file")
    
    for env in environments:
        folder_env = "logs_batch/" + env
        os.system("rm config/lastEnvironment.png")
        os.system("cp environments/" + env + ".png config/lastEnvironment.png")
        os.system("mkdir " + folder_env)
        for n_r in num_robot:
            folder_nr = folder_env + "/" + n_r
            os.system("mkdir " + folder_nr)
            for method in explMethods:
                folder_method = folder_nr + "/" + method[0]
                os.system("mkdir " + folder_method)
                for seed in range(num_run):
                    #aslo team config
                    os.system("rm config/lastTeamConfig.txt")
                    os.system("cp config/teamConfig" + n_r + env + str(seed) + ".txt config/lastTeamConfig.txt")

                    folder_seed = folder_method + "/" +str(seed)
                    os.system("mkdir " + folder_seed)
                    
                    #modify the lastSimulatorConfig.txt
                    compile_config(method, n_r, seed)

                    #call java
                    os.system("java -jar MRESim_4.0.jar")
                    #....compute....#

                    #move all the logs in the proper folder
                    os.system("cp config/lastSimulatorConfig.txt " + folder_seed + "/lastSimulatorConfig.txt")
                    os.system("cp config/lastTeamConfig.txt " + folder_seed + "/lastTeamConfig.txt")
                    os.system("cp config/lastEnvironment.png " + folder_seed + "/lastEnvironment.png")

                    os.system("mv logs/* " + folder_seed)
                    os.system("mv log_file/* " + folder_seed)
