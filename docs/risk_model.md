# Un modello di rischio continuo per l'interazione uomo–robot in simulazione

**Documento tecnico — PeopleFlow / HRISim**

## Sommario

Questo documento descrive la revisione del modulo di calcolo del rischio (`hrisim_risk`) e le modifiche apportate al simulatore pedonale (`pedsim_simulator`) per lo studio di scenari di interazione uomo–robot a velocità variabile. Le modifiche comprendono: (i) l'introduzione di un parametro di velocità degli agenti configurabile a runtime; (ii) la correzione di un difetto geometrico nel rilevamento delle collisioni imminenti, che ne impediva l'attivazione ad alte velocità; (iii) la sostituzione della metrica di rischio con una formulazione continua, adimensionale e normalizzata in $[0,1]$, basata su prossimità e velocità di avvicinamento. La nuova metrica è implementata in modo identico nel nodo online (`risk.py`) e nella pipeline di post-processing (`extract_metrics.py`).

---

## 1. Contesto sperimentale

Lo scenario simulato prevede due agenti in una stanza:

- il **soggetto**, fermo al centro della stanza (waypoint `WP_CENTER`), che abbandona la propria posizione soltanto quando viene rilevata una situazione di collisione imminente;
- l'**ostacolo**, che percorre la stanza muovendosi tra i waypoint dello scenario.

Sono previste due configurazioni speculari, selezionate dal parametro `subject`: umano al centro (`subject:=0`) oppure robot al centro (`subject:=1`). Il medesimo parametro è utilizzato coerentemente da tre componenti:

| Componente | Ruolo del parametro |
|---|---|
| `risk.py` | seleziona chi è soggetto e chi è ostacolo nel calcolo del rischio |
| `PedsimBridge.py` | identifica l'agente da bloccare a `WP_CENTER` fino alla collisione |
| `extract_metrics.py` | replica la selezione soggetto/ostacolo nel post-processing |

Il rischio è quindi sempre riferito al soggetto fermo al centro: misura quanto l'ostacolo gli si sta avvicinando.

## 2. Velocità degli agenti configurabile a runtime

Nel simulatore originale la velocità massima di ciascun pedone era estratta, alla creazione dell'agente, da una distribuzione normale $\mathcal{N}(1.34,\ 0.26)$ m/s (`libpedsim/ped_agent.cpp`) e non era controllabile da alcun parametro. È stato introdotto il parametro `max_agent_speed`:

- `max_agent_speed = 0` (default): comportamento originale, velocità casuale per agente;
- `max_agent_speed > 0`: impone il valore a tutti i pedoni (robot escluso), inclusi quelli generati durante la simulazione.

Il parametro è esposto sia come argomento di lancio (variabile tmule `MAX_AGENT_SPEED`) sia via `dynamic_reconfigure`, quindi è modificabile durante l'esecuzione:

```bash
rosrun dynamic_reconfigure dynparam set /pedsim_simulator max_agent_speed 1.0
# oppure, con interfaccia grafica: rosrun rqt_reconfigure rqt_reconfigure
```

L'applicazione a runtime avviene nel callback di riconfigurazione del simulatore:

```cpp
// simulator.cpp — reconfigureCB
CONFIG.max_agent_speed = config.max_agent_speed;
if (CONFIG.max_agent_speed > 0) {
  for (Agent* agent : SCENE.getAgents()) {
    if (agent->getType() != Ped::Tagent::ROBOT)
      agent->setVmax(CONFIG.max_agent_speed);
  }
}
```

## 3. Rilevamento della collisione imminente

### 3.1 Il modello a cono di velocità

La condizione di collisione imminente è valutata con un test geometrico. Detti $A$ la posizione del soggetto e $B$ quella dell'ostacolo, si costruisce il triangolo (cono) con vertice in $A$ e base costituita dal segmento di semi-larghezza `OBS_SIZE` centrato in $B$ e ortogonale alla congiungente $AB$. Data la velocità relativa

$$
\mathbf{V}_{rel} = \mathbf{v}_B - \mathbf{v}_A,
$$

si proietta il moto relativo di un secondo nel punto $P = A - \mathbf{V}_{rel}$ e si dichiara collisione quando

$$
\text{collision} \iff P \in \text{cono} \ \wedge \ \lVert AB \rVert < d_{safe}.
$$

Intuitivamente: la direzione del moto relativo punta verso l'ostacolo e la distanza è sotto la soglia di sicurezza $d_{safe}$.

### 3.2 Difetto alle alte velocità e correzione

Il test originale utilizzava direttamente $P = A - \mathbf{V}_{rel}$, cioè lo spostamento relativo previsto in un intervallo di 1 s. Poiché la base del cono giace alla distanza dell'ostacolo — per costruzione inferiore a $d_{safe} = 2.3$ m quando il test può scattare — ogni velocità relativa superiore alla distanza corrente produce un punto $P$ **oltre** la base del triangolo: il predicato $P \in \text{cono}$ risulta falso proprio nelle situazioni più pericolose. Con agenti a 3 m/s la collisione non veniva mai rilevata e il soggetto al centro non si spostava più; lo stesso difetto generava una zona cieca anche alle velocità originali, per distanze inferiori a $\lVert\mathbf{V}_{rel}\rVert \cdot 1\,\text{s} \approx 1.3$ m.

La correzione riscala lo spostamento in modo che il punto proiettato non possa superare la base del cono, preservando l'informazione direzionale:

```python
# risk.py / extract_metrics.py — compute_risk
v_rel_norm = math.sqrt(Vrel.x**2 + Vrel.y**2)
dist = subject.distance(obstacle)
scale = min(1.0, 0.9 * dist / v_rel_norm) if v_rel_norm > 0 else 1.0
P = Point(cone_origin.x - Vrel.x * scale, cone_origin.y - Vrel.y * scale)

collision = P.within(cone) and dist < SAFE_DIST
```

Il fattore $0.9$ garantisce che il punto riscalato cada strettamente all'interno del triangolo per ogni direzione ammissibile: la distanza perpendicolare dal vertice alla retta di base è esattamente $\lVert AB\rVert$, quindi ogni punto a distanza $0.9\,\lVert AB\rVert$ dal vertice, con direzione interna all'apertura angolare, appartiene al cono.

**Validazione** (soggetto fermo, ostacolo in avvicinamento frontale):

| Distanza | Velocità | Test originale | Test corretto |
|---:|---:|:---:|:---:|
| 2.2 m | 1.34 m/s | ✅ | ✅ |
| 2.2 m | 3.00 m/s | ❌ | ✅ |
| 1.0 m | 1.34 m/s | ❌ | ✅ |
| 1.0 m | 3.00 m/s | ❌ | ✅ |
| 3.0 m | qualsiasi | ❌ (corretto) | ❌ (corretto) |
| moto tangenziale o in allontanamento | qualsiasi | ❌ (corretto) | ❌ (corretto) |

È stato inoltre corretto un caso di errore non gestito: con i due agenti alla medesima ordinata ($\Delta y = 0$, quindi `slope_AB = 0`) il calcolo della perpendicolare `-1/slope_AB` sollevava `ZeroDivisionError`; il caso è ora gestito analogamente a quello, già coperto, di $\Delta x = 0$.

## 4. La metrica di rischio

### 4.1 Formulazione precedente e limiti

La metrica precedente era definita come

$$
r_{old} = \exp\!\Big(\underbrace{\tfrac{1}{|\Delta x| + |\Delta y|}}_{\text{prossimità } (L_1)} + \; \mathbb{1}_{coll}\cdot\big[\underbrace{\tfrac{\lVert\mathbf{V}_{rel}\rVert}{d}}_{1/TTC} + \underbrace{s_{eff}}_{\text{steering}}\big]\Big)
$$

e presentava i seguenti limiti:

1. **Anisotropia** — la prossimità usava la distanza Manhattan ($L_1$): a parità di distanza euclidea, il rischio dipendeva dall'orientamento della coppia di agenti (fino a un fattore $\sqrt{2}$), in modo incoerente con il resto del modulo ($d_{safe}$ e TTC euclidei).
2. **Scala non limitata e overflow** — il termine $1/d$ diverge al contatto e l'esponenziale amplifica la divergenza: i valori coprivano decine di ordini di grandezza, rendendo il segnale inutilizzabile per statistiche e modelli, con possibilità di `OverflowError` (e perdita del campione) proprio nel quasi-contatto.
3. **Discontinuità temporale** — l'attivazione del flag di collisione sommava istantaneamente i termini $1/TTC$ e steering, introducendo gradini nella serie temporale, dannosi per derivate e analisi causale.
4. **TTC non direzionale** — il tempo alla collisione era calcolato con il modulo $\lVert\mathbf{V}_{rel}\rVert$ anziché con la componente radiale: un transito tangenziale veloce risultava rischioso quanto un avvicinamento frontale alla stessa velocità.
5. **Incoerenza dimensionale** — la somma combinava grandezze in $\text{m}^{-1}$, $\text{s}^{-1}$ e $\text{m}$ con pesi impliciti unitari, dipendenti dalle unità di misura della scena.

### 4.2 Nuova formulazione

Sia $\hat{\mathbf{u}} = (B - A)/d$ il versore dal soggetto all'ostacolo, con $d = \lVert AB \rVert$. Si definiscono:

$$
v_{cl} = \max\big(0,\ -\mathbf{V}_{rel}\cdot\hat{\mathbf{u}}\big)
\qquad\textit{(velocità di avvicinamento)}
$$

$$
r_{prox} = \max\!\Big(0,\ \frac{d_{safe}}{d} - 1\Big)
\qquad
r_{ttc} = \frac{v_{cl}}{d} = \frac{1}{TTC}
$$

$$
\boxed{\ r = 1 - \exp\big(-(w_{p}\, r_{prox} + w_{t}\, r_{ttc})\big) \ \in [0,1]\ }
$$

con caso limite $r = 1$ per $d = 0$. Implementazione (identica nei due file):

```python
# risk.py / extract_metrics.py — compute_risk
if dist > 0:
    ux = (obstacle.x - subject.x) / dist
    uy = (obstacle.y - subject.y) / dist
    v_closing = max(0.0, -(Vrel.x * ux + Vrel.y * uy))
    risk_prox = max(0.0, SAFE_DIST / dist - 1.0)
    risk_ttc  = v_closing / dist
    risk = 1.0 - math.exp(-(W_PROX * risk_prox + W_TTC * risk_ttc))
else:
    risk = 1.0
```

### 4.3 Proprietà

- **Riferita al soggetto**: entrambi i termini sono calcolati rispetto alla posizione del soggetto al centro; $v_{cl}$ misura il tasso di riduzione della distanza dall'ostacolo.
- **Continuità**: nessun termine dipende dal flag binario di collisione; la serie temporale è priva di discontinuità.
- **Limitatezza**: la mappa $x \mapsto 1 - e^{-x}$ comprime il rischio grezzo in $[0,1)$ con saturazione dolce; nessun overflow possibile.
- **Direzionalità**: il clamp $\max(0,\cdot)$ annulla il contributo cinetico quando l'ostacolo si allontana; a parità di distanza, un ostacolo fermo produce un rischio inferiore a uno in avvicinamento.
- **Coerenza dimensionale**: entrambi i termini sono adimensionali una volta fissate le costanti; i pesi $w_p,\ w_t$ sono espliciti (rosparam `/hri/risk_w_prox`, `/hri/risk_w_ttc`, default $1.0$).
- **Località della prossimità**: $r_{prox}$ è nullo oltre $d_{safe}$; il termine $r_{ttc}$ non è invece azzerato a distanza, per conservare l'informazione di avvicinamento anticipato (scelta modificabile applicando il gate di distanza anche a $r_{ttc}$).

### 4.4 Validazione numerica

Transito dell'ostacolo a 3 m/s con distanza minima di 0.4 m dal soggetto fermo ($d_{safe}=2.3$ m, $w_p = w_t = 1$):

| Fase | $d$ | $r$ |
|---|---:|---:|
| avvicinamento da lontano | 6.0 m | 0.39 |
| ingresso nella zona critica | 2.4 m | 0.70 |
| massimo avvicinamento | 0.4 m | 0.99 |
| appena oltre, in allontanamento | 1.3 m | 0.56 |
| fuori dalla zona critica, in allontanamento | 2.4 m | 0.00 |

Casi di controllo: ostacolo fermo a 1 m → $r = 0.73$ (sola prossimità, pari al caso "in allontanamento a 1 m"); ostacolo fermo a 3 m → $r = 0$; moto tangenziale → contributo cinetico nullo.

## 5. Coerenza online / post-processing

La funzione `compute_risk` è replicata testualmente in:

- `HRISim_docker/src/HRISim/hrisim_risk/scripts/risk.py` — nodo ROS che pubblica `/hri/risk` (valore continuo + flag di collisione) a partire dagli stati degli agenti;
- `HRISim_docker/src/HRISim/hrisim_analytics/scripts/extract_metrics.py` — ricalcolo offline dai rosbag, con iniezione di rumore gaussiano su posizioni e velocità ($\sigma = 0.05$).

Le costanti condivise sono riportate nella tabella seguente. La formulazione precedente è conservata, commentata, accanto alla nuova in entrambi i file.

| Parametro | Valore | Definizione |
|---|---:|---|
| `SAFE_DIST` ($d_{safe}$) | 2.3 m | soglia di distanza di sicurezza |
| `OBS_SIZE` | 1.0 m | semi-larghezza della base del cono di collisione |
| `W_PROX` ($w_p$) | 1.0 | peso del termine di prossimità |
| `W_TTC` ($w_t$) | 1.0 | peso del termine cinetico ($1/TTC$) |
| `subject` | 0 / 1 | soggetto al centro: umano (0) o robot (1) |

Il flag di collisione che sblocca il soggetto da `WP_CENTER` (Sezione 3) resta binario e indipendente dal valore continuo di rischio: i consumatori del topic (`PedsimBridge.py`, `risk_visualiser.py`) non richiedono modifiche.
