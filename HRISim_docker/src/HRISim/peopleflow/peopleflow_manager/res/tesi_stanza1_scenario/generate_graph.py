#!/usr/bin/env python3
import networkx as nx
import pickle
import os

def generate_asymmetric_graph():
    G = nx.Graph()

    # Definizione coordinate (devono matchare lo scenario)
    # Agente 1: Zona Nord (y=8)
    # Agente 2: Zona Sud (y=-8)
    waypoints = {
        "WP_A1": (-8.0, 8.0),
        "WP_A2": (8.0, 8.0),
        "WP_B1": (-8.0, -8.0),
        "WP_B2": (8.0, -8.0)
    }

    # Aggiunta nodi con attributo 'pos'
    for name, pos in waypoints.items():
        G.add_node(name, pos=pos)

    # Creazione archi BIDIREZIONALI (Edge) isolati
    # Sottografo A (Nord)
    G.add_edge("WP_A1", "WP_A2", weight=16.0)
    
    # Sottografo B (Sud)
    G.add_edge("WP_B1", "WP_B2", weight=16.0)

    # Salvataggio in formato Pickle
    output_file = "graph.pkl"
    with open(output_file, 'wb') as f:
        pickle.dump(G, f)
    
    print(f"Grafo generato con successo: {os.path.abspath(output_file)}")

if __name__ == "__main__":
    generate_asymmetric_graph()
