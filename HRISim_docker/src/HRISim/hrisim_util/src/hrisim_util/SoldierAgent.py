import rospy
import copy
import random
import networkx as nx
from geometry_msgs.msg import Point
import numpy as np
from hrisim_util.Agent import Agent, DEFAULT_VALUE

class SoldierAgent(Agent):
    def __init__(self, id, schedule, graph, allowTask, maxTaskTime) -> None:
        super().__init__(id, schedule, graph, allowTask, maxTaskTime)
        self.patrol_index = 0

    def selectDestination(self, selected_time, potential_dests):
        # Filtriamo le destinazioni possibili in base allo schedule, 
        # ignorando le probabilità, ci interessa solo quali sono presenti.
        destinations = self.schedule[selected_time]['dests']
        tmp_dest = []
        for dest in potential_dests:
            mean = destinations[dest]['mean']
            if mean > 0:
                tmp_dest.append(dest)

        # Ordiniamo per avere un percorso prevedibile e ripetibile
        sorted_dests = sorted(list(tmp_dest))

        if not sorted_dests:
            return constants.WP.PARKING.value # Fallback sicuro

        # Logica deterministica: scelgo il prossimo nell'elenco
        selected_destination = sorted_dests[self.patrol_index % len(sorted_dests)]

        # Incremento l'indice per il prossimo giro
        self.patrol_index += 1

        rospy.logwarn(f'>>> IL SOLDATO {self.id} E IN MARCIA VERSO: {selected_destination} <<<')
        return selected_destination

    def to_dict(self):
        # Aggiungiamo patrol_index alla serializzazione
        data = super().to_dict()
        data['patrol_index'] = self.patrol_index
        return data

    @classmethod
    def from_dict(cls, data, schedule, graph, allowTask, maxTaskTime):
        # Ripristiniamo patrol_index dalla deserializzazione
        agent = super().from_dict(data, schedule, graph, allowTask, maxTaskTime)
        agent.patrol_index = data.get('patrol_index', 0)
        return agent

    def getTaskDuration(self):
        return 0  # Zero secondi di attesa ai waypoint
