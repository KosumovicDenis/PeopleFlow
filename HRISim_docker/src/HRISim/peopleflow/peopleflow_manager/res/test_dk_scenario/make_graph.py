import networkx as nx
import pickle

G = nx.Graph()

# Assegniamo coordinate REALI anche a Parking e Charging Station
nodi_reali = {
    'wp_TL': (-2.5, 5.0),
    'wp_TR': (2.5, 5.0),
    'wp_BL': (-3.5, -5.0),
    'wp_BR': (3.5, -5.0),
    'parking': (0.0, 6.0),           # In alto al centro
    'charging-station': (0.0, -6.0)  # In basso al centro
}

for name, pos in nodi_reali.items():
    if name in ['parking', 'charging-station']:
        G.add_node(name, pos=pos, name=name, type='station')
    else:
        G.add_node(name, pos=pos, name=name, type='waypoint')

# Connettiamo tutto
all_nodes = list(G.nodes())
for n1 in all_nodes:
    for n2 in all_nodes:
        if n1 != n2:
            G.add_edge(n1, n2, weight=1.0)

with open('graph.pkl', 'wb') as f:
    pickle.dump(G, f)

print("Grafo 6.0: Il Parcheggio è ora un luogo reale!")
