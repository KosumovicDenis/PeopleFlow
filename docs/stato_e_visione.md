# Stato del progetto e visione

**Ultimo aggiornamento: 2026-07-14**

Questo documento fotografa lo stato della pipeline sperimentale e le decisioni
metodologiche prese, per valutare rapidamente dove siamo e cosa resta.
Dettagli tecnici: [risk_model.md](risk_model.md) (modello di rischio) e
[workflow_esperimenti.md](workflow_esperimenti.md) (procedura operativa).

---

## 1. Visione

L'obiettivo della tesi è un **trigger rapido di "causal drift"**: un test che,
osservando lo stream di dati di un'interazione uomo–robot, rilevi quando il
*modello causale* sottostante è cambiato (e quindi serve una nuova causal
discovery), senza scattare per semplici cambi di regime (es. velocità diverse).

Banco di prova: due scenari simulati **speculari** — un agente fermo al centro
che si sposta quando rileva una collisione imminente, e un agente che percorre
la stanza — dove tra scenario 1 e 2 cambia solo *chi* ricopre quale ruolo
(umano/robot). I DAG dei due scenari, scoperti con F-PCMCI, fanno da ground
truth strutturale; il test MMD sui residui di un predittore fa da trigger.

```
   pedsim/Gazebo ──rosbag──> estrazione ──CSV──> F-PCMCI  (DAG di riferimento)
        │                        │
   (registrazioni)               └──────────────> test MMD (trigger di drift)
```

## 2. Risultati chiave (in ordine cronologico)

1. **Velocità degli agenti configurabile** (`max_agent_speed`, override
   per-agente `agent0/1_speed`, tutto via dynamic_reconfigure) e **fix del
   rilevamento collisione** ad alte velocità (il test del cono falliva per
   overshoot del punto di lookahead).
2. **Formula del rischio riscritta**: continua, adimensionale, normalizzata in
   [0,1] (prossimità + inverso del TTC), identica online e in post-processing.
3. **Analisi di identificabilità**: con la velocità del mover imposta
   (costante), l'arco `V_mover → Risk` non è identificabile (causa senza
   varianza) e la geometria latente genera archi spuri. Introdotta
   l'**eccitazione del mover** (`speed_excitation.py`) come intervento
   esogeno.
4. **Scoperta decisiva**: per la causal discovery la variabile giusta non è
   `Risk` ma la **distanza `D`** — `V→D` è cinematica pura (funziona anche a
   velocità costante, senza eccitazione), accoppiamenti ~2× più forti, nessuna
   formula da giustificare. `Risk` resta la variabile del test MMD.
5. **Risultato finale sui DAG** (4 registrazioni gemelle da 21 min,
   scenario × velocità {1.5, 2.0} m/s): nucleo replicato in tutte le run,
   **speculare tra scenari e stabile tra velocità**:
   - Scenario 1: `Vr → D`, `D → Vh`, `D → Vr`
   - Scenario 2: `Vh → D`, `D → Vr`, `D → Vh`
   Il ruolo si legge dall'arco che entra in `D` (dal mover); `D → V_centro` è
   la reazione (l'arco di interazione); `D → V_mover` è reale ma mediato dalla
   posizione (il tour è un programma spaziale). Archi residui deboli e
   intermittenti: `V_centro → D` (il centro è quasi sempre fermo) e un diretto
   marginale in 1 run su 4.
6. **Test MMD validato** (design corretto): addestrando il predittore e
   calibrando la banda nulla su *tutta la variabilità tollerata* (entrambe le
   velocità), la separazione tra cambio di regime (MMD ≈ 0.004) e cambio di
   struttura (MMD ≈ 0.069) è **~15×**. Il design originale (training a una
   sola velocità) produceva falsi allarmi 87× sopra soglia per covariate
   shift/estrapolazione fuori supporto.

## 3. Stato dei componenti

| Componente | Stato | Dove |
|---|---|---|
| Simulatore (velocità, override, eccitazione) | ✅ completo e compilato | `pedsim_simulator`, `hrisim_recording/scripts/speed_excitation.py` |
| Rilevamento collisione (cono) | ✅ corretto e validato | `hrisim_risk/scripts/risk.py` |
| Formula rischio continua | ✅ online + offline | `risk.py`, `extract_metrics.py` |
| Estrazione diretta dai bag (no replay, secondi) | ✅ | `hrisim_analytics/scripts/extract_from_bag.py` |
| Estrazione via replay (legacy, robusta) | ✅ | `extract_metrics.py`, `process_bag.sh` |
| DAG speculari (variabile `D`) | ✅ risultato ottenuto | CausalFlowDocker: `ws/final_D_graphs.py`, PNG `cm_D_scenario*` |
| Test MMD di drift | ✅ validato su dati brevi | CausalFlowDocker: `ws/drift_test.py` |
| Registrazioni definitive | ✅ 4 gemelle 21 min + 2 eccitate | `shared/*.bag` |

## 4. Decisioni metodologiche e razionale

- **`max_lag = 1` (vincolo esterno)**: le scale temporali si portano al passo
  del modello con decimazione ×4 (0.4 s, tarata sulla latenza di reazione
  misurata) — per `D` nessuno shift (la posizione integra le velocità), per le
  formule di rischio shift di +1 (calcolo istantaneo).
- **`D` per la discovery, `Risk` per il drift**: la discovery premia variabili
  con accoppiamento forte alle osservate; il trigger premia sensibilità alla
  conditional del meccanismo. Due scopi, due variabili.
- **Eccitazione solo per i dataset del test MMD** (copertura del range di
  velocità in una run); per i DAG è dannosa (il livello esogeno resta impresso
  nella traiettoria della distanza).
- **Niente shift markoviani manuali** nel CSV: erano tarati sulla vecchia
  formula (che incorporava la velocità del soggetto); i lag ora sono gestiti
  in analisi.
- **`Collision` fuori dal modello causale**: variabile binaria con test
  pensati per continue (TE gaussiana/GPDC) degrada l'intero grafo; resta nel
  CSV come ground truth del trigger.

## 5. Prossimi passi

1. Rieseguire il test MMD sui 4 CSV lunghi (controllo negativo 1.5→2.0 con 3×
   i dati; positivo sc1→sc2) e fissare la taratura della soglia (banda nulla
   con transizione inclusa, oppure 8σ).
2. Figure definitive per la tesi: 2 DAG-intersezione (uno per scenario) +
   tabella p-value per run + grafico MMD con soglia e cambio scenario.
3. Scrittura: i capitoli di metodo possono partire da `risk_model.md` e
   `workflow_esperimenti.md`; la discussione dei limiti da §4 di questo file.

## 6. Limiti noti (da discutere in tesi)

- `D → V_mover` è una dipendenza reale mediata dalla posizione (latente):
  inevitabile finché il tour del mover è un percorso spaziale fisso.
- `V_centro → D` è fisicamente debole (il centro è fermo ~66% del tempo):
  appare solo in alcune run; si riporta come arco intermittente.
- I p-value GPDC hanno granularità grossolana vicino alla soglia; le
  conclusioni si basano sul nucleo replicato in tutte le run, non su singoli
  test borderline.
