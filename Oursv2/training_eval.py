# THIS MODULE WILL EVALUATE IN EACH ITERATION SOME METRICS IN ORDER TO KNOW HOW WELL THE MODEL IS PERFORMING
# VARIABLES
grasp_ratios = [] # TO SAVE ALL GRASP RATIOS WHEN TABLE IS CLEARED
push_ratios = [] # TO SAVE ALL PUSH RATIOS WHEN TABLE IS CLEARED
grasp_ratio = 0 # KEEPS GRASP TO ALL RATIO VALUE
push_ratio = 0 # KEEPS PUSH TO ALL RATIO VALUE

class train_eval():
    def __init__(self):
        pass

    # EVALUATION METHOD CALLED AS STATIC METHOD
    @classmethod
    def evaluate_iter(self, iteration, all_grasps, succ_grasps, all_pushes, succ_pushes, num_obj_complete, primitive_action):
        num_obj_complete = num_obj_complete # NUMBER OF OBJECTS TO USE FOR TESTING IF WE ARE GETTING A GOOD ACTION EFFICIENCY
        total_grasps = all_grasps # TOTAL NUMBER OF GRASPS
        total_pushes = all_pushes # TOTAL NUMBER OF PUSHES
        grasps_succ = succ_grasps # NUMBER OF SUCCESSFUL GRASPS
        pushes_succ = succ_pushes # NUMBER OF SUCCESSFUL PUSHES

        global grasp_ratio
        global grasp_ratios
        global push_ratio
        global push_ratios

        print("*"*50)
        print("Evaluating...")
        print("Total pushes made until now: " + str(total_pushes))
        print("Total grasps made until now: " + str(total_grasps))
        print("Successful grasps until now: " + str(grasps_succ))
        print("Successful pushes until now: " + str(pushes_succ))

        if primitive_action == 'grasp':
            grasp_ratio = self.grasp_to_all_ratio(total_grasps, grasps_succ) # OBTAIN GRASP TO ALL RATIO
            grasp_ratios.append(grasp_ratio)
            push_ratios.append(push_ratio)
            print("Grasp to All Ratio: " + str(grasp_ratio) + "%")
            print("Push to All Ratio: " + str(push_ratio) + "%")
        else:
            print("Grasp to All Ratio: " + str(grasp_ratio) + "%")
            push_ratio = self.push_to_all_ratio(total_pushes, pushes_succ) # OBTAIN PUSH TO ALL RATIO
            grasp_ratios.append(grasp_ratio)
            push_ratios.append(push_ratio)
            print("Push to All Ratio: " + str(push_ratio) + "%")

        print("*"*50)

    # OBTAIN THE AVG PUSH TO ACTIONS RATIO (IN THE UNIVERSE OF ALL PUSHES)
    @classmethod
    def push_to_all_ratio(self, total_pushes, pushes_succ):
        total_p = total_pushes  # TOTAL NUMBER OF PUSHES
        p_succ = pushes_succ    # NUMBER OF SUCCESSFUL PUSHES

        if total_p == 0:
            return 0
        else:
            return round((p_succ/total_p*100),2)
    
    # OBTAIN THE AVG GRASP TO ACTIONS RATIO (IN THE UNIVERSE OF ALL GRASPS)
    @classmethod
    def grasp_to_all_ratio(self, total_grasps, grasps_succ):
        total_g = total_grasps  # TOTAL NUMBER OF GRASPS
        g_succ = grasps_succ    # NUMBER OF SUCCESSFUL GRASPS

        if grasps_succ == 0:
            return 0
        else:
            return round((g_succ/total_g*100),2)
    
    # GET PUSH RATIOS
    @classmethod
    def get_push_ratios(self):
        global push_ratios
        return push_ratios

    # GET GRASP RATIOS
    @classmethod
    def get_grasp_ratios(self):
        global grasp_ratios
        return grasp_ratios