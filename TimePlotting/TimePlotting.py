import matplotlib.pyplot as plt
import csv
import time
import Config

class TimePlotter():
    def __init__(self,pr_graph):
        self.pr_graph = pr_graph
        self.no_sequences = None
        self.no_total_nodes = 0
        self.percent_nodes_refined = [0]
        self.prob_refined = [0]
        self.start_time = time.time()
        self.times = [0]
        with open(Config.PROJ_DIR+"ICRA_Results/Delicate_cans/nodes_" + Config.RESULTS_FILE, "a") as f:
            f.write("\n")
        with open(Config.PROJ_DIR+"ICRA_Results/Delicate_cans/prob_" + Config.RESULTS_FILE, "a") as f:
            f.write("\n")
        with open(Config.PROJ_DIR+"ICRA_Results/Delicate_cans/time_" + Config.RESULTS_FILE, "a") as f:
            f.write("\n")

        self.init()

    def init(self):
        root_node = self.pr_graph.get_root_node()
        self.no_sequences = root_node.lqueue.qsize()
        queue = root_node.lqueue.queue
        for seq in queue:
            hlas = seq[1]
            self.no_total_nodes += hlas.length

    def update(self,pr_node):
        t = time.time()
        min_sequences = pr_node.lqueue.queue
        remaining_nodes = 0
        remaining_probs = 0
        for seq in min_sequences:
            hlas = seq[1]
            remaining_nodes += hlas.length
            edge =pr_node.hl_plan_tree.get_edge(hlas.action_list[-2], hlas.action_list[-1])
            if edge.ll_plan is None:
                remaining_probs += edge.prob
        percent_nodes_refined = ((self.no_total_nodes - remaining_nodes) / float(self.no_total_nodes)) * 100
        self.percent_nodes_refined.append(percent_nodes_refined)
        self.prob_refined.append(1-remaining_probs)
        self.times.append(t - self.start_time)
        with open(Config.PROJ_DIR+"ICRA_Results/Delicate_cans/nodes_" + Config.RESULTS_FILE, "a") as f:
            f.write(str(percent_nodes_refined)+",")
        with open(Config.PROJ_DIR+"ICRA_Results/Delicate_cans/prob_" + Config.RESULTS_FILE, "a") as f:
            f.write(str(1-remaining_probs) + ",")
        with open(Config.PROJ_DIR+"ICRA_Results/Delicate_cans/time_" + Config.RESULTS_FILE, "a") as f:
            f.write(str(t - self.start_time) + ",")

    def generate_plot(self):
        # with open("nodes_"+Config.RESULTS_FILE,"a") as f:
        #     writer = csv.writer(f)
        #     writer.writerow(self.percent_nodes_refined)
        # with open("prob_"+Config.RESULTS_FILE,"a") as f:
        #     writer = csv.writer(f)
        #     writer.writerow(self.prob_refined)
        # with open("time_"+Config.RESULTS_FILE,"a") as f:
        #     writer = csv.writer(f)
        #     writer.writerow(self.times)
        pass
        # f = plt.figure()
        # plt.plot(self.percent_nodes_refined,self.prob_refined)
        # plt.xlabel("Percentage of nodes refined")
        # plt.ylabel("Probability mass refined")
        # plt.savefig("result.png")



