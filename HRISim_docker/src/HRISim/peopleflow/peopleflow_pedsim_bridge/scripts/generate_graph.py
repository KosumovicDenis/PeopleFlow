#!/usr/bin/env python3
import networkx as nx
import pickle
import os

def generate_asymmetric_graph():
    G = nx.Graph()

    # Definizione coordinate per stanza 10x10 (limiti +/- 5)
    # Agente 1: Zona Nord (y=4)
    # Agente 2: Zona Sud (y=-4)
    waypoints = {
        "WP_A1": (-4.0, 4.0),
        "WP_A2": (4.0, 4.0),
        "WP_B1": (-4.0, -4.0),
        "WP_B2": (4.0, -4.0)
    }

    # Aggiunta nodi con attributo 'pos'
    for name, pos in waypoints.items():
        G.add_node(name, pos=pos)

    # Creazione archi BIDIREZIONALI (Edge) isolati
    # Sottografo A (Nord) - Distanza 8 metri
    G.add_edge("WP_A1", "WP_A2", weight=8.0)
    
    # Sottografo B (Sud) - Distanza 8 metri
    G.add_edge("WP_B1", "WP_B2", weight=8.0)

    # Salvataggio in formato Pickle nel percorso corretto per il bridge
    # Nota: Il bridge cerca in peopleflow_manager/res/<scenario>/graph.pkl
    output_dir = os.path.join(os.path.dirname(__file__), "../../../peopleflow_manager/res/tesi_stanza1_scenario")
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "graph.pkl")
    
    with open(output_file, 'wb') as f:
        pickle.dump(G, f)
    
    print(f"Grafo (10x10) generato con successo in: {os.path.abspath(output_file)}")

if __name__ == "__main__":
    generate_asymmetric_graph()
