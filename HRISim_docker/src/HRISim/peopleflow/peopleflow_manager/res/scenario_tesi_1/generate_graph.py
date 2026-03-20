#!/usr/bin/env python3
import networkx as nx
import pickle
import os
import math

def dist(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def generate_tesi_1_graph():
    G = nx.Graph()

    # Coordinate dei waypoint (devono corrispondere all'XML dello scenario)
    waypoints = {
        "WP_CENTER": (0.0, 0.0),
        "WP_SHELF": (3.0, -3.0),
        "WP_BOXES": (-2.5, 2.5)
    }

    # Aggiunta nodi
    for name, pos in waypoints.items():
        G.add_node(name, pos=pos)

    # Connessioni (Tutti collegati tra loro in una stanza aperta)
    G.add_edge("WP_CENTER", "WP_SHELF", weight=dist(waypoints["WP_CENTER"], waypoints["WP_SHELF"]))
    G.add_edge("WP_CENTER", "WP_BOXES", weight=dist(waypoints["WP_CENTER"], waypoints["WP_BOXES"]))
    G.add_edge("WP_SHELF", "WP_BOXES", weight=dist(waypoints["WP_SHELF"], waypoints["WP_BOXES"]))

    # Salvataggio
    output_dir = os.path.dirname(os.path.abspath(__file__))
    output_file = os.path.join(output_dir, "graph.pkl")
    with open(output_file, 'wb') as f:
        pickle.dump(G, f)
    
    print(f"Grafo 'scenario_tesi_1' generato con successo in: {output_file}")

if __name__ == "__main__":
    generate_tesi_1_graph()
