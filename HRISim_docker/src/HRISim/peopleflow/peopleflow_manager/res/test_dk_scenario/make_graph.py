import networkx as nx
import pickle

G = nx.Graph()

# Nodi reali
nodi_reali = {
    'wp_TL': (-2.5, 5.0),
    'wp_TR': (2.5, 5.0),
    'wp_BL': (-2.5, -5.0),
    'wp_BR': (2.5, -5.0)
}
for name, pos in nodi_reali.items():
    G.add_node(name, pos=pos, name=name, type='waypoint')

# Nodi fantasma
G.add_node('charging-station', pos=(0.0, 0.0), name='charging-station', type='station')
G.add_node('parking', pos=(0.0, 0.0), name='parking', type='station')

# IL FIX: Colleghiamo TUTTI i nodi tra loro per far funzionare il Traveling Salesman Problem!
all_nodes = list(G.nodes())
for n1 in all_nodes:
    for n2 in all_nodes:
        if n1 != n2:
            G.add_edge(n1, n2, weight=1.0)

with open('graph.pkl', 'wb') as f:
    pickle.dump(G, f)

print("Grafo 4.0: Tutti i nodi connessi. TIAGo_plan è felice!")
