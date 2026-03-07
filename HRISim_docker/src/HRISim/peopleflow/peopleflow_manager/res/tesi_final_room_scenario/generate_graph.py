#!/usr/bin/env python3
import networkx as nx
import pickle
import os
import math

def dist(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def generate_final_graph():
    G = nx.Graph()

    # Definizione di tutti i waypoint (inclusi punti di transito per i varchi)
    waypoints = {
        # STANZA 1
        "WP_A1": (-4.0, 4.0),
        "WP_A2": (4.0, 4.0),
        "WP_B1": (-4.0, -4.0),
        "WP_B2": (4.0, -4.0),
        "WP_DOOR_1": (4.5, 0.0),   # Varco Stanza 1
        
        # CORRIDOIO
        "WP_MID": (7.5, 0.0),      # Centro Corridoio
        
        # STANZA 2
        "WP_DOOR_2": (10.5, 0.0),  # Varco Stanza 2
        "WP_C1": (11.0, 4.0),
        "WP_C2": (19.0, 4.0),
        "WP_D1": (11.0, -4.0),
        "WP_D2": (19.0, -4.0)
    }

    # Aggiunta nodi con attributo posizione
    for name, pos in waypoints.items():
        G.add_node(name, pos=pos)

    # CONNESSIONI STANZA 1
    # Percorsi agenti
    G.add_edge("WP_A1", "WP_A2", weight=dist(waypoints["WP_A1"], waypoints["WP_A2"]))
    G.add_edge("WP_B1", "WP_B2", weight=dist(waypoints["WP_B1"], waypoints["WP_B2"]))
    # Connessioni al varco per il robot
    for wp in ["WP_A1", "WP_A2", "WP_B1", "WP_B2"]:
        G.add_edge(wp, "WP_DOOR_1", weight=dist(waypoints[wp], waypoints["WP_DOOR_1"]))

    # CONNESSIONE CORRIDOIO
    G.add_edge("WP_DOOR_1", "WP_MID", weight=dist(waypoints["WP_DOOR_1"], waypoints["WP_MID"]))
    G.add_edge("WP_MID", "WP_DOOR_2", weight=dist(waypoints["WP_MID"], waypoints["WP_DOOR_2"]))

    # CONNESSIONI STANZA 2
    # Percorsi agenti
    G.add_edge("WP_C1", "WP_C2", weight=dist(waypoints["WP_C1"], waypoints["WP_C2"]))
    G.add_edge("WP_D1", "WP_D2", weight=dist(waypoints["WP_D1"], waypoints["WP_D2"]))
    # Connessioni al varco per il robot
    for wp in ["WP_C1", "WP_C2", "WP_D1", "WP_D2"]:
        G.add_edge("WP_DOOR_2", wp, weight=dist(waypoints["WP_DOOR_2"], waypoints[wp]))

    # Salvataggio del file graph.pkl
    output_dir = os.path.dirname(os.path.abspath(__file__))
    output_file = os.path.join(output_dir, "graph.pkl")
    with open(output_file, 'wb') as f:
        pickle.dump(G, f)
    
    print(f"Grafo navigazione generato con successo!")
    print(f"Waypoint totali: {len(G.nodes)}")
    print(f"Connessioni totali: {len(G.edges)}")
    print(f"File: {output_file}")

if __name__ == "__main__":
    generate_final_graph()
